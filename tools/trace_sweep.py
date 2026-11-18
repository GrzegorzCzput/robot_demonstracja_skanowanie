"""Trace a new side-to-side camera path with IK (Jan's geometry, wider at the ends).

Same construction as Jan's plan_scan/scan_geometry: PCHIP radius, height and roll
over the view azimuths, camera aimed at the object center, gravity-referenced roll.
"""
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import PchipInterpolator

sys.path.insert(0, str(Path(__file__).parent))
from ik_tools import CENTER, clearances, ik, roll_of, camera  # noqa: E402

STEP = 0.5  # degrees


def camera_path(knots, radii, heights, rolls, az):
    r = PchipInterpolator(knots, radii)(az)
    h = PchipInterpolator(knots, heights)(az)
    roll = PchipInterpolator(knots, rolls)(az)
    a = np.radians(az)
    return np.column_stack([CENTER[0] + r * np.cos(a), CENTER[1] + r * np.sin(a), h]), roll


def trace(end_az, r_end, h_end, roll_end, seed_mid, verbose=False):
    knots = [end_az, 150., 210., 360. - end_az]
    radii, heights = [r_end, 0.28, 0.28, r_end], [h_end, 0.34, 0.34, h_end]
    rolls = [-roll_end, 0., 0., roll_end]
    az = np.unique(np.r_[np.arange(knots[0], knots[-1] + 1e-9, STEP), knots])
    pos, roll = camera_path(knots, radii, heights, rolls, az)
    mid = int(np.argmin(np.abs(az - 180.)))
    q = np.zeros((len(az), 6))
    for order in (range(mid, len(az)), range(mid, -1, -1)):
        seed = seed_mid
        for i in order:
            sol, pe, re = ik(pos[i], roll[i], seed)
            if pe > 1e-5 or re > 0.01:
                return None, f"IK failed at {az[i]:.1f} deg (pos {pe * 1000:.2f} mm, rot {re:.2f} deg)"
            if np.abs(sol - seed).max() > 0.15 and i != mid:
                return None, f"joint jump {np.abs(sol - seed).max():.2f} rad at {az[i]:.1f} deg"
            q[i], seed = sol, sol
    cl = np.array([clearances(x) for x in q])
    rolls_actual = np.array([roll_of(x) for x in q])
    info = dict(min_object_mm=1000 * cl[:, 0].min(), min_self_mm=1000 * cl[:, 2].min(),
                min_table_mm=1000 * cl[:, 1].min(), max_roll=np.abs(rolls_actual).max())
    if cl[:, 0].min() < 0.011 or cl[:, 2].min() < 0.020 or np.abs(rolls_actual).max() > 20.01:
        return None, f"constraint: {info}"
    views = [int(np.argmin(np.abs(az - k))) for k in knots]
    return dict(q=q, az=az, pos=pos, views=views, knots=knots, radii=radii, heights=heights,
                rolls=rolls, **info), "ok"


if __name__ == "__main__":
    scan = np.load(Path(__file__).parent.parent / "outputs/scan_4_positions.npz")
    # Jan's middle pose (azimuth 180) seeds the trace.
    sq = scan["position"]
    cams = np.array([camera(x)[:3, 3] for x in sq[:len(sq) // 2]])
    seed_mid = sq[int(np.argmin(np.abs(cams[:, 1])))]
    results = []
    for r_end in (0.30, 0.29, 0.28):
        for h_end in (0.28, 0.29, 0.30, 0.31):
            for roll_end in (18., 20.):
                res, msg = trace(125., r_end, h_end, roll_end, seed_mid)
                print(f"r {100 * r_end:.0f} h {100 * h_end:.0f} roll {roll_end:.0f}: "
                      + (f"OK  object {res['min_object_mm']:.1f} mm, self {res['min_self_mm']:.1f} mm, "
                         f"max roll {res['max_roll']:.1f}" if res else msg), flush=True)
                if res:
                    results.append((r_end, -roll_end, res['min_object_mm'], h_end, res))
        if results:
            break
    best = max(results, key=lambda x: (x[0], x[1], x[2]))[-1]
    print("chosen:", {k: best[k] for k in ("knots", "radii", "heights", "rolls", "min_object_mm", "min_self_mm", "max_roll")})
    np.savez_compressed(Path(__file__).parent.parent / "outputs/sweep_path_125.npz", position=best["q"],
                        view_indices=np.array(best["views"]), azimuth_deg=best["az"],
                        camera_position=best["pos"], joint_names=scan["joint_names"],
                        velocity_limits=scan["velocity_limits"], acceleration_limits=scan["acceleration_limits"],
                        object_center=scan["object_center"], knots=best["knots"], radii=best["radii"],
                        heights=best["heights"], rolls=best["rolls"])
