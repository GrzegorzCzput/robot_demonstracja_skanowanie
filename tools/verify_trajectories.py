import sys, json
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))  # needs follow_trajectory.py from main next to it
from ik_tools import camera, clearances, roll_of, CENTER, LIMITS
import follow_trajectory as ft
path = np.load(Path(__file__).parent.parent / "outputs/sweep_path_125.npz"); arc = np.load(Path(__file__).parent.parent / "outputs/scan_upward_arc.npz")
ref = np.vstack([path["camera_position"], [camera(x)[:3, 3] for x in arc["position"]]])
ref = np.vstack([np.linspace(a, b, 6) for a, b in zip(ref[:-1], ref[1:]) if np.linalg.norm(b - a) < .02])
targets = np.vstack([path["camera_position"][path["view_indices"]], [camera(x)[:3, 3] for x in arc["position"][arc["view_indices"]]]])
for f in sys.argv[1:]:
    t, q, qd, loop, smax, dwell = ft.load(Path(f))      # also enforces the driver joint-limit buffer
    d = np.load(f); v, a = d["velocity"], d["acceleration"]; vl, al = d["velocity_limits"], d["acceleration_limits"]
    fd = np.gradient(v, t, axis=0)
    M = [camera(x) for x in q]; c = np.array([m[:3, 3] for m in M])
    aim = np.array([np.degrees(np.arccos(np.clip(m[:3, 0] @ (CENTER - m[:3, 3]) / np.linalg.norm(CENTER - m[:3, 3]), -1, 1))) for m in M])
    roll = np.array([roll_of(x) for x in q[::2]])
    dev = np.array([np.linalg.norm(ref - x, axis=1).min() for x in c])
    cl = np.array([clearances(x) for x in q[::2]])
    margin = np.minimum(q - (LIMITS[:, 0] + 0.05), (LIMITS[:, 1] - 0.05) - q).min()
    print(f"{Path(f).name}: cycle {t[-1]:.2f} s, closed {loop}, max speed {smax:.2f}x | v/cap {np.max(np.abs(v)/vl):.3f} "
          f"a/cap {np.max(np.abs(a)/al):.3f} (fd {np.max(np.abs(fd)/al):.3f})")
    print(f"   aim max {aim.max():.2f} deg | roll max {np.abs(roll).max():.1f} deg | camera off planned paths max {1000*dev.max():.1f} mm"
          f" | joint-limit margin beyond driver clamp {margin:.3f} rad")
    print(f"   clearance (vertex): object {1000*cl[:,0].min():.1f} mm, table {1000*cl[:,1].min():.1f} mm, self {1000*cl[:,2].min():.1f} mm")
    same = np.all(q[1:] == q[:-1], axis=1)
    if same.any():
        idx = [i for i in range(1, len(same)) if same[i] and not same[i-1]] + ([0] if same[0] else [])
        err = [1000 * np.linalg.norm(targets - c[i], axis=1).min() for i in idx]
        print(f"   {len(idx)} stops, distance to planned photo positions max {max(err):.3f} mm;"
              f" sweep end distance from object {[round(np.linalg.norm(c[i][:2] - CENTER[:2])*100, 1) for i in idx if abs(c[i][1]) > 0.2]} cm")
