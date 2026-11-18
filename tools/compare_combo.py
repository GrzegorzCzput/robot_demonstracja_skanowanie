"""Compare combo versions: slowdown at the switches, aim, path deviation, clearance."""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, sys.argv[3])
from fk import Robot

scan, arcf = np.load(sys.argv[1]), np.load(sys.argv[2])
robot = Robot(Path(sys.argv[4]))
center, half, TABLE = scan["object_center"], np.full(3, .1), -0.03
cam = lambda q: robot.fk(q)["camera"]
vi = scan["view_indices"]
ref_q = np.vstack([scan["position"][vi[0]:vi[3] + 1], arcf["position"]])
ref_cam = np.array([cam(x)[:3, 3] for x in ref_q])
ref_dense = np.vstack([np.linspace(a, b, 8) for a, b in zip(ref_cam[:-1], ref_cam[1:]) if np.linalg.norm(b - a) < .02])


def clearance(q):
    p = robot.triangles(q).reshape(-1, 3)
    d = np.abs(p - center) - half
    return (np.linalg.norm(np.clip(d, 0, None), axis=1) + np.minimum(d.max(1), 0)).min()


for f in sys.argv[5:]:
    d = np.load(f); t, q, v, a = d["time"], d["position"], d["velocity"], d["acceleration"]
    vl, al = d["velocity_limits"], d["acceleration_limits"]
    c = np.array([cam(x)[:3, 3] for x in q])
    cs = np.linalg.norm(np.gradient(c, t, axis=0), axis=1)          # camera speed (m/s)
    mid = np.abs(c[:, 1]) < 0.06                                       # near the switch region (middle)
    fd = np.gradient(v, t, axis=0)
    aim = np.array([np.degrees(np.arccos(np.clip(cam(x)[:3, 0] @ (center - cam(x)[:3, 3]) / np.linalg.norm(center - cam(x)[:3, 3]), -1, 1))) for x in q])
    dev = np.array([np.linalg.norm(ref_dense - x, axis=1).min() for x in c])
    clr = min(clearance(x) for x in q[::3])
    # stretches near the middle where the camera is below 30% of its top speed
    slow = (cs < 0.3 * cs.max()) & mid & (np.abs(c[:, 1]) < 0.06)
    print(f"{Path(f).name}: cycle {t[-1]:.2f} s | v/cap {np.max(np.abs(v)/vl):.3f} a/cap {np.max(np.abs(a)/al):.3f} "
          f"fd a/cap {np.max(np.abs(fd)/al):.3f} closed {np.array_equal(q[0], q[-1])}")
    print(f"   camera top speed {cs.max():.2f} m/s; near the middle: time below 30% speed {slow.sum()*0.02:.2f} s, "
          f"min camera speed while switching {cs[mid & (c[:, 2] > 0.30) & (c[:, 2] < 0.37)].min():.3f} m/s")
    print(f"   camera off Jan's paths: max {1000*dev.max():.1f} mm | aim error max {aim.max():.2f} deg | object clearance {1000*clr:.1f} mm")
