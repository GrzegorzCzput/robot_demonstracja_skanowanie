# Continuous sweep tools

`make_sweep.py` turns the forward path of `outputs/scan_4_positions.npz` into a
continuous side-to-side sweep: constant speed along the planned path, braking
only to reverse at the two ends. `verify_sweep.py` checks it against the
original plan (limits, joint and robot-surface deviation, camera aim).

```bash
python3 tools/make_sweep.py outputs/scan_4_positions.npz outputs/sweep.npz 0.03
cp render_upward_arc.py tools/fk.py   # FK helper for the check
python3 tools/verify_sweep.py outputs/scan_4_positions.npz outputs/sweep.npz tools \
    third_party/trlc-dk1/urdf/follower/TRLC-DK1-Follower.urdf
```

The 0.03 rad thinning removes rounding kinks at the old stop points. Result:
5.65 s cycle, 92% velocity / 95% acceleration of the caps, at most 0.8 mm
robot-surface deviation from the planned poses, 0.001 deg aim error.
The robot repo's `main` branch plays it as `trajectory/sweep.npz`.
