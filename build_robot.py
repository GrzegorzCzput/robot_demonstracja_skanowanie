"""Build a cuRobo model using the official TRLC-DK1 wrist camera."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent


def build():
    from curobo.robot_builder import RobotBuilder
    import torch

    np.random.seed(123)
    torch.manual_seed(123)

    cfg = yaml.safe_load((ROOT / "config/demo.yaml").read_text())
    source = ROOT / cfg["robot_urdf"]
    tree = ET.parse(source)
    robot = tree.getroot()
    # Absolute mesh paths allow the generated URDF to live in config/.
    for mesh in robot.findall(".//mesh"):
        mesh.set("filename", str((source.parent / mesh.get("filename")).resolve()))
    # Keep the unused gripper fixed; the original wrist camera frame is retained.
    for joint in robot.findall("joint"):
        if joint.get("name", "").startswith("gripper_") and joint.get("type") == "prismatic":
            joint.set("type", "fixed")
            joint.remove(joint.find("limit"))
    output = ROOT / "config/trlc_dk1_camera.urdf"
    tree.write(output, encoding="utf-8", xml_declaration=True)

    builder = RobotBuilder(str(output), str(source.parent), tool_frames=[cfg["camera_frame"]])
    builder.fit_collision_spheres(use_collision_mesh=True, sphere_density=1.5, iterations=150)
    spheres = builder.collision_spheres.copy()
    # Build uses immediate neighbors only. Do not auto-ignore pairs just because
    # they collide in the URDF's arbitrary zero configuration.
    model = builder.build()
    builder.save(model, str(ROOT / "config/trlc_dk1.yaml"), include_cspace=False)
    data = yaml.safe_load((ROOT / "config/trlc_dk1.yaml").read_text())
    kin = data["kinematics"]
    kin["collision_spheres"] = spheres
    kin["collision_link_names"] = list(spheres)
    kin["mesh_link_names"] = list(spheres)
    kin["self_collision_ignore"].update({
        # Sphere approximations protrude around the fixed shoulder and wrist
        # housings. These two-hop assembly neighbors share the same joint area.
        "base_link": ["link1-2", "link2-3"],
        "link3-4": ["link2-3", "link4-5", "link5-6"],
        "link4-5": ["link5-6", "link6-7"],
        "link5-6": ["link6-7", "finger_left", "finger_right"],
        "link6-7": ["finger_left", "finger_right"],
        "finger_left": ["finger_right"],
    })
    kin["cspace"] = {
        "joint_names": [f"joint{i}" for i in range(1, 7)],
        "default_joint_position": [0.0, 1.4, 2.3, 0.3, 0.0, 0.0],
        "max_acceleration": cfg["acceleration_reference_rad_s2"],
        "max_jerk": [100.0] * 6,
        "null_space_weight": [1.0] * 6,
        "cspace_distance_weight": [1.0] * 6,
    }
    (ROOT / "config/trlc_dk1.yaml").write_text(yaml.safe_dump(data, sort_keys=False))
    print(f"Built {output}; {sum(map(len, spheres.values()))} collision spheres", flush=True)


if __name__ == "__main__":
    build()
