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

## Combo (sweep, then arc)

`make_combo.py` builds one cycle: sweep middle -> end A -> end B -> middle, a
0.2 s switch onto the upward arc at about 40 deg (the paths are 0.04 rad apart
there), arc up -> bottom -> back to 40 deg, and the switch back.

```bash
python3 tools/make_combo.py outputs/scan_4_positions.npz outputs/scan_upward_arc.npz outputs/combo.npz
python3 tools/verify_combo.py outputs/scan_4_positions.npz outputs/scan_upward_arc.npz outputs/combo.npz \
    tools third_party/trlc-dk1/urdf/follower/TRLC-DK1-Follower.urdf
```

Result: 10.7 s cycle, 92% velocity / 95% acceleration of the caps, at most
0.0023 rad from the planned paths, vertex-based object clearance 11.0 mm (same
as the original plans), switch moves over 11 cm from the object.
