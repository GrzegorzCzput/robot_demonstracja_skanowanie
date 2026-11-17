"""Play the wide scan or the upward-only camera arc in Viser."""
import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import viser
from viser.extras import ViserUrdf
import yaml

from scan_geometry import MODE_LABELS, trajectory_file

ROOT = Path(__file__).resolve().parent


@dataclass
class Trajectory:
    data: dict
    report: dict
    rotations: Rotation
    optical_quaternions: np.ndarray
    joint_order: list

    @classmethod
    def load(cls, path, joint_names):
        with np.load(path, allow_pickle=False) as archive:
            data = {key: archive[key] for key in archive.files}
        report = json.loads(str(data["metadata_json"]))
        rotations = Rotation.from_quat(data["camera_quaternion_wxyz"], scalar_first=True)
        # URDF camera: X forward, Z up. Viser frustum: Z forward, Y down.
        optical = Rotation.from_matrix(np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]))
        order = [list(data["joint_names"]).index(name) for name in joint_names]
        return cls(data, report, rotations, (rotations * optical).as_quat(scalar_first=True), order)


@dataclass
class Playback:
    mode: str
    requested_mode: str
    time: float = 0.
    last_gui_time: float = 0.
    last_index: int = -1
    restart_requested: bool = False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path, help="Override the selected mode's NPZ")
    parser.add_argument("--mode", choices=tuple(MODE_LABELS), default="scan")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--host", default="127.0.0.1")
    args = parser.parse_args()
    paths = {mode: trajectory_file(ROOT, mode) for mode in MODE_LABELS}
    if args.trajectory is not None:
        paths[args.mode] = args.trajectory
    for path in paths.values():
        if not path.exists():
            raise FileNotFoundError(f"Missing {path}. Generate both modes with: .venv/bin/python plan_scan.py --mode all")

    server = viser.ViserServer(host=args.host, port=args.port)
    server.scene.set_up_direction("+z")
    server.scene.add_grid("/grid", width=1.5, height=1.5, cell_size=0.1, position=(0.35, 0., -0.03))
    robot = ViserUrdf(server, ROOT / "config/trlc_dk1_camera.urdf", root_node_name="/robot")
    trajectories = {mode: Trajectory.load(path, robot.get_actuated_joint_names()) for mode, path in paths.items()}
    active = trajectories[args.mode]
    playback = Playback(args.mode, args.mode)

    world = yaml.safe_load((ROOT / "config/world.yaml").read_text())
    for name, box in world["cuboid"].items():
        if name == "scan_object":
            continue
        mesh = trimesh.creation.box(extents=box["dims"])
        mesh.visual.face_colors = (170, 135, 90, 255)
        server.scene.add_mesh_trimesh(f"/world/{name}", mesh, position=tuple(box["pose"][:3]))

    target = server.scene.add_frame("/target", axes_length=0.055, axes_radius=0.002)
    overlays = [target]
    path_handle = server.scene.add_line_segments("/camera_path", points=np.zeros((1, 2, 3)),
                                                 colors=(50, 195, 235), line_width=3.)
    markers = []
    for i in range(4):
        frame = server.scene.add_frame(f"/views/{i + 1}", axes_length=0.045, axes_radius=0.0015)
        server.scene.add_label(f"/views/{i + 1}/label", text=f"View {i + 1}",
                               position=(0., -0.07 - i * .035, .035 * i))
        ray = server.scene.add_line_segments(f"/view_rays/{i + 1}", points=np.zeros((1, 2, 3)),
                                              colors=(235, 175, 55), line_width=1.5)
        markers.append((frame, ray))
        overlays.extend([frame, ray])
    camera = server.scene.add_camera_frustum("/robot_camera", fov=np.deg2rad(60), aspect=16/9,
                                              scale=0.07, color=(70, 210, 140))
    aim = server.scene.add_line_segments("/aim", points=np.zeros((1, 2, 3)),
                                         colors=(70, 210, 140), line_width=2.)
    down_axis = server.scene.add_line_segments("/image_down_axis", points=np.zeros((1, 2, 3)),
                                               colors=(170, 80, 230), line_width=5.)
    down_label = server.scene.add_label("/image_down_label", text="Image down")
    overlays.extend([camera, aim, down_axis, down_label])

    title = server.gui.add_markdown("# TRLC-DK1 scanning demo")
    mode_control = server.gui.add_dropdown("Motion mode", options=tuple(MODE_LABELS.values()),
                                           initial_value=MODE_LABELS[args.mode])
    play = server.gui.add_checkbox("Play", initial_value=True)
    loop = server.gui.add_checkbox("Loop", initial_value=False)
    speed = server.gui.add_slider("Playback speed", min=0.1, max=1., step=0.1, initial_value=1.)
    slider = server.gui.add_slider("Time (s)", min=0., max=float(active.data["time"][-1]),
                                   step=float(active.report["sample_dt"]), initial_value=0.)
    restart = server.gui.add_button("Restart")
    viewpoint = server.gui.add_dropdown("Viewpoint", options=("Scene", "Robot camera"), initial_value="Scene")
    phase = server.gui.add_markdown("")
    summary = server.gui.add_markdown("")
    path_visible = server.gui.add_checkbox("Show trajectory", initial_value=True)

    def overlay_visibility():
        for overlay in overlays:
            overlay.visible = viewpoint.value == "Scene"
        path_handle.visible = path_visible.value and viewpoint.value == "Scene"

    def activate(mode):
        nonlocal active
        active = trajectories[mode]
        data, report = active.data, active.report
        xyz = data["camera_position"]
        with server.atomic():
            playback.mode = mode
            playback.time = playback.last_gui_time = 0.
            playback.last_index = -1
            playback.restart_requested = False
            slider.max = float(data["time"][-1])
            slider.step = float(report["sample_dt"])
            slider.value = 0.
            loop.value = bool(report["loop_allowed"])
            loop.disabled = not report["loop_allowed"]
            play.value = True
            target.position = tuple(data["object_center"])
            step = max(1, len(xyz) // 2500)
            points = xyz[::step]
            path_handle.points = np.stack([points[:-1], points[1:]], axis=1)
            for (frame, ray), idx in zip(markers, data["view_indices"]):
                frame.position = tuple(xyz[idx])
                frame.wxyz = tuple(data["camera_quaternion_wxyz"][idx])
                ray.points = np.array([[xyz[idx], data["object_center"]]])
            if mode == "scan":
                span = np.ptp(report["view_angles_deg"])
                description = f"Four-view scan · {span:.0f}° sweep · {report['duration_seconds']:.0f} s loop"
                behavior = "Returns along the viewing arc."
            else:
                description = f"Upward arc · {100*report['arc_radius_m']:.0f} cm from target · {report['duration_seconds']:.0f} s"
                behavior = ("Upward only; stops at the top. Restart resets; Play replays.  \n"
                            f"**Top elevation:** {report['view_parameters_deg'][-1]:.1f}° "
                            f"(reachable limit {report['reachable_elevation_limit_deg']:.1f}°)")
            title.content = f"# TRLC-DK1 scanning demo\n{description} · four {report['dwell_seconds']:.0f} s captures"
            summary.content = (
                f"{behavior}  \n"
                f"**Picture roll:** ±{report['max_abs_camera_roll_deg']:.1f}° maximum  \n"
                f"**Velocity cap:** {report['velocity_limit_rad_s'][0]:.2f} rad/s  \n"
                f"**Acceleration cap:** {report['acceleration_limit_rad_s2'][0]:.2f} rad/s²  \n"
                "Acceleration uses an assumed demo reference.  \n"
                f"**Closest world clearance:** {1000*report['min_world_clearance_m']:.1f} mm")
            overlay_visibility()

    @mode_control.on_update
    def _(_event):
        playback.requested_mode = next(mode for mode, label in MODE_LABELS.items() if label == mode_control.value)

    @restart.on_click
    def _(_event):
        playback.restart_requested = True

    @path_visible.on_update
    def _(_event):
        overlay_visibility()

    def scene_camera(client):
        client.camera.position = (1.1, -1.1, 0.85)
        client.camera.look_at = (0.25, 0., 0.23)
        client.camera.up_direction = (0., 0., 1.)

    @server.on_client_connect
    def _(client):
        scene_camera(client)

    @viewpoint.on_update
    def _(_event):
        overlay_visibility()
        if viewpoint.value == "Scene":
            for client in server.get_clients().values():
                scene_camera(client)

    activate(args.mode)
    previous = time.monotonic()
    print(f"Viser ready: http://{args.host}:{args.port} — two motion modes", flush=True)
    while True:
        now = time.monotonic()
        elapsed, previous = now - previous, now
        if playback.requested_mode != playback.mode:
            activate(playback.requested_mode)
        if playback.restart_requested:
            playback.time = 0.
            playback.restart_requested = False
            slider.value = playback.last_gui_time = 0.
            playback.last_index = -1
        data, report = active.data, active.report
        if abs(slider.value - playback.last_gui_time) > 1e-8:
            playback.time = slider.value
        if play.value:
            playback.time += elapsed * speed.value
            if playback.time >= data["time"][-1]:
                if report["loop_allowed"] and loop.value:
                    playback.time %= float(data["time"][-1])
                else:
                    playback.time = float(data["time"][-1])
                    play.value = False
        idx = min(int(playback.time / report["sample_dt"]), len(data["time"]) - 1)
        if idx != playback.last_index:
            xyz = data["camera_position"][idx]
            with server.atomic():
                robot.update_cfg(data["position"][idx, active.joint_order])
                camera.position = tuple(xyz)
                camera.wxyz = tuple(active.optical_quaternions[idx])
                aim.points = np.array([[xyz, data["object_center"]]])
                down_tip = xyz + 0.12 * data["image_down_world"][idx]
                down_axis.points = np.array([[xyz, down_tip]])
                down_label.position = tuple(down_tip)
                raw = str(data["phase"][idx])
                text = f"Simulated capture {raw[-1]}" if raw.startswith("view_") else raw.replace("_", " ")
                if playback.time >= data["time"][-1] and not report["loop_allowed"]:
                    text = "Complete — stopped at top"
                phase.content = (f"**{text.title()}** · {data['time'][idx]:.2f} / {data['time'][-1]:.2f} s  \n"
                                 f"Picture roll: {data['camera_roll_deg'][idx]:+.1f}°")
                slider.value = float(data["time"][idx])
                playback.last_gui_time = slider.value
            playback.last_index = idx
        if viewpoint.value == "Robot camera":
            xyz = data["camera_position"][idx]
            rotation = active.rotations[idx].as_matrix()
            for client in server.get_clients().values():
                client.camera.position = tuple(xyz)
                client.camera.look_at = tuple(xyz + rotation[:, 0])
                client.camera.up_direction = tuple(rotation[:, 2])
        time.sleep(1 / 60)


if __name__ == "__main__":
    main()
