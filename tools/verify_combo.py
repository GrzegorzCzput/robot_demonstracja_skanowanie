"""Check combo.npz against Jan's plans: limits, path deviation, aim, mesh clearance."""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, sys.argv[4])
from fk import Robot

scan, arcf, combo = (np.load(p) for p in sys.argv[1:4])
robot = Robot(Path(sys.argv[5]))
center = combo["object_center"]
q, v, a, t = combo["position"], combo["velocity"], combo["acceleration"], combo["time"]
vl, al = combo["velocity_limits"], combo["acceleration_limits"]
report = json.loads(str(combo["metadata_json"]))

fd_v, fd_a = np.gradient(q, t, axis=0), np.gradient(v, t, axis=0)
print(f"peak v/cap {np.max(np.abs(v) / vl):.3f}, a/cap {np.max(np.abs(a) / al):.3f}, "
      f"finite-diff a/cap {np.max(np.abs(fd_a) / al):.3f}, v-vs-fd rel err "
      f"{np.linalg.norm(fd_v - v) / np.linalg.norm(v):.4f}, closed {np.array_equal(q[0], q[-1])}")

# Which samples belong to the two switch moves?
legs = list(report["legs"].values())
starts = np.cumsum([0] + legs)
switch = ((t >= starts[3]) & (t <= starts[4])) | ((t >= starts[7]) & (t <= starts[8]))

vi = scan["view_indices"]
ref = np.vstack([scan["position"][vi[0]:vi[3] + 1], arcf["position"]])
dense = np.vstack([np.linspace(x, y, 10) for x, y in zip(ref[:-1], ref[1:])
                   if np.abs(y - x).max() < 0.2])
dj = np.array([np.abs(dense - x).max(1).min() for x in q[~switch]])
print(f"path parts: max joint deviation from Jan's paths {dj.max():.4f} rad")

TABLE_TOP = -0.03
half = np.array([0.1, 0.1, 0.1])


def clearance(qq):
    pts = robot.triangles(qq).reshape(-1, 3)
    table = pts[:, 2].min() - TABLE_TOP
    dlt = np.abs(pts - center) - half
    box = (np.linalg.norm(np.clip(dlt, 0, None), axis=1) + np.minimum(dlt.max(1), 0)).min()
    return table, box


def aim(qq):
    m = robot.fk(qq)["camera"]
    d = center - m[:3, 3]
    return np.degrees(np.arccos(np.clip(m[:3, 0] @ d / np.linalg.norm(d), -1, 1)))


jan = np.array([clearance(x) for x in np.vstack([scan["position"][::6], arcf["position"][::6]])])
sw = np.array([clearance(x) for x in q[switch]])
whole = np.array([clearance(x) for x in q[::4]])
print(f"mesh clearance (vertex-based) on Jan's trajectories: table {1000 * jan[:, 0].min():.1f} mm, "
      f"object {1000 * jan[:, 1].min():.1f} mm")
print(f"mesh clearance on the switch moves:                  table {1000 * sw[:, 0].min():.1f} mm, "
      f"object {1000 * sw[:, 1].min():.1f} mm")
print(f"mesh clearance on the whole combo cycle:             table {1000 * whole[:, 0].min():.1f} mm, "
      f"object {1000 * whole[:, 1].min():.1f} mm")
print(f"camera aim error: path parts max {max(aim(x) for x in q[~switch][::3]):.3f} deg, "
      f"switch moves max {max(aim(x) for x in q[switch]):.3f} deg")
