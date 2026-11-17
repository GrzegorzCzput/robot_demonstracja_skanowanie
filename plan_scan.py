"""Generate one collision-checked, closed four-view cuRobo scanning trajectory."""
import json
import argparse
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation

from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import GoalToolPose, JointState, Pose

from trajectory_math import retime
from scan_geometry import camera_path, trajectory_file, view_parameters

ROOT = Path(__file__).resolve().parent


def goal_for(position, target, roll_deg=0.):
    # Official camera convention: +X optical axis, +Z up, +Y left.
    x = target - position
    x /= np.linalg.norm(x)
    y = np.cross([0., 0., 1.], x)
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    rotation = np.column_stack([x, y, z]) @ Rotation.from_euler("x", roll_deg, degrees=True).as_matrix()
    quat = Rotation.from_matrix(rotation).as_quat(scalar_first=True)
    pose = Pose(position=torch.tensor(position[None], device="cuda", dtype=torch.float32),
                quaternion=torch.tensor(quat[None], device="cuda", dtype=torch.float32))
    return GoalToolPose.from_poses({"camera": pose}, num_goalset=1), quat


def validate_geometry(planner, q, scene, margin=0.):
    """Check every saved sample using the same cuRobo FK spheres and pair mask."""
    pairs = planner.kinematics.config.self_collision_config.collision_pairs.long()
    min_world, min_self = float("inf"), float("inf")
    positions, quaternions = [], []
    for chunk in np.array_split(q, max(1, int(np.ceil(len(q) / 256)))):
        state = JointState.from_position(torch.tensor(chunk, device="cuda", dtype=torch.float32),
                                         joint_names=planner.joint_names)
        kin = planner.compute_kinematics(state)
        spheres = kin.robot_spheres.reshape(len(chunk), -1, 4)
        centers, radius = spheres[..., :3], spheres[..., 3]
        valid = radius > 0
        for box in scene["cuboid"].values():
            center = torch.tensor(box["pose"][:3], device="cuda")
            half = torch.tensor(box["dims"], device="cuda") / 2
            delta = torch.abs(centers - center) - half
            sdf = torch.linalg.vector_norm(torch.clamp(delta, min=0), dim=-1) + torch.clamp(delta.amax(-1), max=0)
            clear = (sdf - radius)[valid].min().item()
            min_world = min(min_world, clear)
        if len(pairs):
            left, right = spheres[:, pairs[:, 0]], spheres[:, pairs[:, 1]]
            clear = torch.linalg.vector_norm(left[..., :3] - right[..., :3], dim=-1) - left[..., 3] - right[..., 3]
            mask = (left[..., 3] > 0) & (right[..., 3] > 0)
            min_self = min(min_self, clear[mask].min().item())
        pose = kin.tool_poses.get_link_pose("camera")
        positions.append(pose.position.detach().cpu().numpy().reshape(-1, 3))
        quaternions.append(pose.quaternion.detach().cpu().numpy().reshape(-1, 4))
    if min_world < margin - 1e-6 or min_self < margin - 1e-6:
        raise RuntimeError(f"Final path collision: world clearance={min_world:.6f} m, self clearance={min_self:.6f} m")
    return np.concatenate(positions), np.concatenate(quaternions), min_world, min_self


def plan_camera_guide(planner, cfg, center, start, mode):
    """Follow a gravity-referenced camera arc with collision-aware cuRobo IK.

    Warm-start every pose from its neighbor and select the nearest feasible
    solution. The return retraces the same arc so it cannot take a wrist-flip
    shortcut. These guide samples introduce no additional picture pauses.
    """
    view_angles = view_parameters(cfg, mode)
    step = cfg["camera_guide_step_deg"] if mode == "scan" else cfg["upward_arc"]["guide_step_deg"]
    n = int(np.ceil((view_angles[-1] - view_angles[0]) / step))
    angles = np.unique(np.r_[np.linspace(view_angles[0], view_angles[-1], n + 1), view_angles])
    positions, rolls = camera_path(cfg, mode, angles)
    current = start
    samples = [start.position.detach().cpu().numpy().reshape(6).astype(float)]
    for i, angle in enumerate(angles[1:], 1):
        goal, _ = goal_for(positions[i], center, float(rolls[i]))
        result = planner.ik_solver.solve_pose(goal, current_state=current, return_seeds=8)
        candidates = result.js_solution.position.reshape(-1, 6)
        valid = result.success.reshape(-1)
        if not valid.any():
            raise RuntimeError(f"Gravity-referenced camera guide is unreachable at {angle:.2f} degrees")
        candidates = candidates[valid]
        costs = torch.linalg.vector_norm(candidates - current.position.reshape(1, 6), dim=-1)
        selected = candidates[costs.argmin()].detach().clone()
        if torch.max(torch.abs(selected - current.position.reshape(6))).item() > 0.3:
            raise RuntimeError(f"IK branch discontinuity at camera angle {angle:.2f} degrees")
        current = JointState.from_position(selected[None], joint_names=planner.joint_names)
        samples.append(selected.cpu().numpy().astype(float))
        if i % 10 == 0 or i == len(angles) - 1:
            print(f"Gravity-referenced camera guide {i}/{len(angles) - 1}", flush=True)
    samples = np.asarray(samples)
    indices = [int(np.flatnonzero(np.isclose(angles, a))[0]) for a in view_angles]
    paths = [samples[indices[i]:indices[i + 1] + 1].copy() for i in range(3)]
    if mode == "scan":
        paths.append(samples[::-1].copy())
    return paths


def resolve_upward_limit(planner, cfg, scene, seed):
    """Find the highest continuously reachable elevation on this fixed arc."""
    settings = cfg["upward_arc"]
    start = settings["start_elevation_deg"]
    ceiling = settings["max_elevation_search_deg"]
    center = np.asarray(cfg["object_center"], dtype=float)

    def solve(elevation, current):
        positions, rolls = camera_path(cfg, "upward_arc", [elevation])
        goal, desired_quat = goal_for(positions[0], center, float(rolls[0]))
        result = planner.ik_solver.solve_pose(goal, current_state=current, return_seeds=8)
        candidates = result.js_solution.position.reshape(-1, 6)[result.success.reshape(-1)]
        if not len(candidates):
            return None
        costs = torch.linalg.vector_norm(candidates - current.position.reshape(1, 6), dim=-1)
        for idx in torch.argsort(costs)[:3]:
            candidate = candidates[idx].detach().clone()
            try:
                xyz, quat, _, _ = validate_geometry(planner, candidate.cpu().numpy()[None], scene)
            except RuntimeError:
                continue
            position_error = np.linalg.norm(xyz[0] - positions[0])
            rotation_error = (Rotation.from_quat(desired_quat, scalar_first=True).inv() *
                              Rotation.from_quat(quat[0], scalar_first=True)).magnitude()
            # Tight FK checks keep the solver's usual goal tolerance from
            # counting an approximate, physically unreachable pose as success.
            if position_error <= 1e-5 and rotation_error <= 1e-4:
                return JointState.from_position(candidate[None], joint_names=planner.joint_names)
        return None

    current = solve(start, seed)
    if current is None:
        raise RuntimeError("The upward arc's starting pose is not collision-free and reachable")
    low, high = start, ceiling
    failed = False
    for elevation in np.r_[np.arange(start + settings["limit_search_step_deg"], ceiling,
                                     settings["limit_search_step_deg"]), ceiling]:
        candidate = solve(float(elevation), current)
        if candidate is None:
            high = float(elevation)
            failed = True
            break
        low, current = float(elevation), candidate
        print(f"Upward reachability search: {low:.2f} degrees reachable", flush=True)
    if failed:
        while high - low > settings["limit_search_tolerance_deg"]:
            middle = (low + high) / 2
            candidate = solve(middle, current)
            if candidate is None:
                high = middle
            else:
                low, current = middle, candidate
    endpoint = low - settings["endpoint_margin_deg"]
    if endpoint <= start:
        raise RuntimeError("No upward motion remains after applying the endpoint margin")
    settings["view_elevations_deg"] = np.linspace(start, endpoint, settings["num_views"]).tolist()
    settings["reachable_limit_deg"] = low
    settings["first_unreachable_elevation_deg"] = high if failed else None
    settings["search_ceiling_reached"] = not failed
    print(f"Upward limit: {low:.3f} degrees; planned top: {endpoint:.3f} degrees", flush=True)


def plan_mode(mode):
    torch.manual_seed(123)
    cfg = yaml.safe_load((ROOT / "config/demo.yaml").read_text())
    center = np.asarray(cfg["object_center"], dtype=float)
    scene = {"cuboid": {
        "table": {"dims": [1.4, 1.2, 0.05], "pose": [0.35, 0., -0.055, 1., 0., 0., 0.]},
        "scan_object": {"dims": cfg["object_size"], "pose": [*cfg["object_center"], 1., 0., 0., 0.]},
    }}
    (ROOT / "config/world.yaml").write_text(yaml.safe_dump(scene))
    planner = MotionPlanner(MotionPlannerCfg.create(
        robot=str(ROOT / "config/trlc_dk1.yaml"), scene_model=scene,
        use_cuda_graph=False, num_ik_seeds=64, num_trajopt_seeds=4,
        optimizer_collision_activation_distance=0.015, random_seed=123,
        position_tolerance=0.002, orientation_tolerance=0.02))
    print("cuRobo planner initialized", flush=True)
    seed = JointState.from_position(planner.default_joint_state.position.reshape(1, -1), joint_names=planner.joint_names)
    if mode == "upward_arc" and cfg["upward_arc"]["auto_max_elevation"]:
        resolve_upward_limit(planner, cfg, scene, seed)
    parameters = view_parameters(cfg, mode)
    views, view_rolls = camera_path(cfg, mode, parameters)
    goals, view_quats = zip(*(goal_for(p, center, roll) for p, roll in zip(views, view_rolls)))
    ik = planner.ik_solver.solve_pose(goals[0], current_state=seed)
    if not ik.success.any():
        raise RuntimeError(f"First view IK failed: success={ik.success}, position_error={ik.position_error}")
    q0 = ik.js_solution.position.reshape(-1, 6)[0].detach().clone()
    current = JointState.from_position(q0[None], joint_names=planner.joint_names)
    print(f"First view: {q0.cpu().tolist()}", flush=True)
    source = ET.parse(ROOT / cfg["robot_urdf"]).getroot()
    limits = {j.get("name"): j.find("limit") for j in source.findall("joint") if j.find("limit") is not None}
    vmax = np.array([float(limits[n].get("velocity")) for n in planner.joint_names])
    lower = np.array([float(limits[n].get("lower")) for n in planner.joint_names])
    upper = np.array([float(limits[n].get("upper")) for n in planner.joint_names])
    amax = np.asarray(cfg["acceleration_reference_rad_s2"])
    vl, al = vmax * cfg["velocity_scale"], amax * cfg["acceleration_scale"]
    dt = cfg["sample_dt"]
    pause_n = int(round(cfg["dwell_seconds"] / dt))
    qs, qds, qdds, phases, view_indices = [], [], [], [], []
    count = 0
    paths = plan_camera_guide(planner, cfg, center, current, mode)
    minimum_moves = [retime(path, vl, al, dt, parameterization="arc_length")[-1] for path in paths]
    available_n = int(round(cfg["target_duration_seconds"] / dt)) - 4 * pause_n
    minimum_n = np.rint(np.asarray(minimum_moves) / dt).astype(int)
    if minimum_n.sum() > available_n:
        raise RuntimeError(f"Views require {sum(minimum_moves) + 4 * pause_n * dt:.2f}s at the current limits; target is {cfg['target_duration_seconds']}s")
    extra = available_n - minimum_n.sum()
    shares = extra * minimum_n / minimum_n.sum()
    move_n = minimum_n + np.floor(shares).astype(int)
    for i in np.argsort(-(shares - np.floor(shares)))[:available_n - move_n.sum()]:
        move_n[i] += 1
    for leg, path in enumerate(paths):
        start = path[0]
        view_indices.append(count)
        qs.append(np.tile(start, (pause_n, 1)))
        qds.append(np.zeros((pause_n, 6)))
        qdds.append(np.zeros((pause_n, 6)))
        phases.extend([f"view_{leg + 1}"] * pause_n)
        count += pause_n
        q, qd, qdd, spline, duration = retime(path, vl, al, dt, requested_duration=move_n[leg] * dt,
                                             parameterization="arc_length")
        # Include start, omit end: next dwell supplies the endpoint at the next timestamp.
        qs.append(q[:-1]); qds.append(qd[:-1]); qdds.append(qdd[:-1])
        phases.extend([f"move_{leg + 1}_to_{(leg + 1) % 4 + 1}"] * (len(q) - 1))
        count += len(q) - 1
        print(f"Leg {leg + 1}: {len(path)} planner samples; retimed to {duration:.2f}s", flush=True)
    terminal = paths[-1][-1]
    if mode == "upward_arc":
        view_indices.append(count)
        qs.append(np.tile(terminal, (pause_n, 1))); qds.append(np.zeros((pause_n, 6))); qdds.append(np.zeros((pause_n, 6)))
        phases.extend(["view_4"] * pause_n)
    qs.append(terminal[None].astype(float)); qds.append(np.zeros((1, 6))); qdds.append(np.zeros((1, 6)))
    phases.append("view_1" if mode == "scan" else "view_4")
    q, qd, qdd = map(np.concatenate, (qs, qds, qdds))
    t = np.arange(len(q)) * dt
    if np.any(q < lower - 1e-6) or np.any(q > upper + 1e-6):
        raise RuntimeError("Smoothed trajectory exceeds joint position limits")
    if np.any(np.abs(qd) > vl + 1e-8) or np.any(np.abs(qdd) > al + 1e-8):
        raise RuntimeError("Smoothed trajectory exceeds scaled derivative limits")
    xyz, quat, world_clear, self_clear = validate_geometry(planner, q, scene)
    rotations = Rotation.from_quat(quat, scalar_first=True).as_matrix()
    optical_axis = rotations[:, :, 0]
    gravity_up = np.array([0., 0., 1.]) - optical_axis * optical_axis[:, 2, None]
    gravity_up /= np.linalg.norm(gravity_up, axis=1)[:, None]
    camera_roll = np.rad2deg(np.arctan2(np.sum(rotations[:, :, 1] * gravity_up, axis=1),
                                       np.sum(rotations[:, :, 2] * gravity_up, axis=1)))
    image_down = -rotations[:, :, 2]
    down_alignment = -image_down[:, 2]
    roll_limit = cfg["max_camera_roll_deg"] if mode == "scan" else cfg["upward_arc"]["max_camera_roll_deg"]
    minimum_down = .3 if mode == "scan" else .02
    if np.max(np.abs(camera_roll)) > roll_limit or np.min(down_alignment) < minimum_down:
        raise RuntimeError(f"Camera image-down constraint failed: roll={np.max(np.abs(camera_roll)):.2f} degrees, down alignment={np.min(down_alignment):.3f}")
    all_directions = center - xyz
    all_directions /= np.linalg.norm(all_directions, axis=1)[:, None]
    all_aim_errors = np.rad2deg(np.arccos(np.clip(np.sum(optical_axis * all_directions, axis=1), -1., 1.)))
    if all_aim_errors.max() > cfg["max_continuous_aim_error_deg"]:
        raise RuntimeError(f"Camera lost target between views: {all_aim_errors.max():.3f} degrees")
    distance = np.linalg.norm(xyz - center, axis=1)
    if mode == "upward_arc":
        radius_error = float(np.max(np.abs(distance - cfg["upward_arc"]["radius"])))
        if radius_error > .001 or np.min(np.diff(xyz[:, 2])) < -2e-5:
            raise RuntimeError(f"Upward arc constraint failed: radius error={radius_error:.6f}m; minimum Z step={np.min(np.diff(xyz[:, 2])):.6f}m")
    actual_views = xyz[view_indices]
    if np.max(np.linalg.norm(actual_views - views, axis=1)) > 0.003:
        raise RuntimeError(f"View position mismatch: {actual_views - views}")
    forward = Rotation.from_quat(quat[view_indices], scalar_first=True).apply(np.tile([1., 0., 0.], (4, 1)))
    direction = center - actual_views
    direction /= np.linalg.norm(direction, axis=1)[:, None]
    aim_error = np.rad2deg(np.arccos(np.clip(np.sum(forward * direction, axis=1), -1, 1)))
    if np.max(aim_error) > 2.:
        raise RuntimeError(f"Camera pointing mismatch: {aim_error}")
    report = dict(planner="cuRobo collision-aware IK with a gravity-referenced Cartesian guide",
                  mode=mode, loop_allowed=mode == "scan",
                  samples=len(q), duration_seconds=float(t[-1]), sample_dt=dt,
                  camera_frame="camera", camera_optical_axis="+X", dwell_seconds=cfg["dwell_seconds"],
                  target_duration_seconds=cfg["target_duration_seconds"], move_durations_seconds=(move_n * dt).tolist(),
                  view_parameters_deg=parameters.tolist(), view_rolls_deg=view_rolls.tolist(),
                  view_heights_m=views[:, 2].tolist(),
                  view_radii_m=np.linalg.norm(views[:, :2] - center[:2], axis=1).tolist(),
                  camera_guide_step_deg=cfg["camera_guide_step_deg"] if mode == "scan" else cfg["upward_arc"]["guide_step_deg"],
                  max_abs_camera_roll_deg=float(np.abs(camera_roll).max()),
                  camera_roll_limit_deg=roll_limit,
                  min_image_down_alignment=float(down_alignment.min()),
                  max_continuous_aim_error_deg=float(all_aim_errors.max()),
                  image_down_axis="camera -Z (optical image +Y)",
                  return_path="reverse viewing arc, no joint-space shortcut" if mode == "scan" else "none; stop at top",
                  peak_velocity_rad_s=np.abs(qd).max(axis=0).tolist(),
                  peak_acceleration_rad_s2=np.abs(qdd).max(axis=0).tolist(),
                  velocity_limit_rad_s=vl.tolist(), acceleration_limit_rad_s2=al.tolist(),
                  velocity_scale=cfg["velocity_scale"], acceleration_scale=cfg["acceleration_scale"],
                  acceleration_reference_source=cfg["acceleration_reference_source"],
                  min_world_clearance_m=world_clear, min_self_clearance_m=self_clear,
                  view_pointing_error_deg=aim_error.tolist(),
                  closed_loop=bool(np.array_equal(q[0], q[-1])),
                  robot_source_commit=subprocess.check_output(["git", "-C", str(ROOT / "third_party/trlc-dk1"), "rev-parse", "HEAD"], text=True).strip(),
                  retiming="C2 cubic path + septic time blend; continuous Bernstein derivative bounds",
                  collision_validation="all saved samples, fitted robot spheres; configured neighbor exclusions")
    if mode == "scan":
        report["view_angles_deg"] = parameters.tolist()
    else:
        report.update(arc_radius_m=cfg["upward_arc"]["radius"],
                      arc_azimuth_deg=cfg["upward_arc"]["azimuth_deg"],
                      max_radius_error_m=radius_error,
                      reachable_elevation_limit_deg=cfg["upward_arc"]["reachable_limit_deg"],
                      first_unreachable_elevation_deg=cfg["upward_arc"]["first_unreachable_elevation_deg"],
                      endpoint_margin_deg=cfg["upward_arc"]["endpoint_margin_deg"],
                      maximum_camera_height_m=float(xyz[:, 2].max()),
                      minimum_camera_z_step_m=float(np.min(np.diff(xyz[:, 2]))))
    output = trajectory_file(ROOT, mode)
    np.savez_compressed(output, time=t, position=q, velocity=qd, acceleration=qdd,
                        joint_names=np.asarray(planner.joint_names), phase=np.asarray(phases),
                        view_indices=np.asarray(view_indices), view_joint_positions=q[view_indices],
                        view_positions=views, view_quaternions_wxyz=np.array(view_quats),
                        camera_position=xyz, camera_quaternion_wxyz=quat,
                        camera_roll_deg=camera_roll, image_down_world=image_down,
                        velocity_limits=vl, acceleration_limits=al,
                        robot_max_velocity=vmax, acceleration_reference=amax,
                        object_center=center, metadata_json=np.array(json.dumps(report)))
    report_name = "validation.json" if mode == "scan" else "validation_upward_arc.json"
    (ROOT / "outputs" / report_name).write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)
    print(f"Saved {output}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("scan", "upward_arc", "all"), default="all")
    args = parser.parse_args()
    for mode in ("scan", "upward_arc") if args.mode == "all" else (args.mode,):
        print(f"Planning mode: {mode}", flush=True)
        plan_mode(mode)


if __name__ == "__main__":
    main()
