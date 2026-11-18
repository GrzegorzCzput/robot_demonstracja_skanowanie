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

`make_combo.py` builds one cycle starting at sweep end A: across to end B, back
to the middle, curve onto the upward arc at about 40 deg, up to the top, down to
the bottom, back to 40 deg, curve back onto the sweep and out to end A.

The two paths use different joints (sweep: 1, 5, 6; arc: 2, 3, 4) and meet at
90 deg in joint space, so the switch overlaps the braking of one with the
speed-up of the other instead of stopping. The 0.04 rad gap between the paths
is closed by a quintic over the same window. The last argument scales the
overlap; 1.0 (ramps fully overlapped) is used. Larger values keep a little more
speed but cut the corner by 5-15 cm with 1-10 deg aim error.

```bash
python3 tools/make_combo.py outputs/scan_4_positions.npz outputs/scan_upward_arc.npz outputs/combo.npz 1.0

python3 tools/compare_combo.py outputs/scan_4_positions.npz outputs/scan_upward_arc.npz tools \
    third_party/trlc-dk1/urdf/follower/TRLC-DK1-Follower.urdf outputs/combo.npz   # switch smoothness
```

Result: 9.5 s cycle, 92% velocity / 95% acceleration of the caps, slowest camera
speed at the switches 0.09 m/s (no stop), camera at most 8.6 mm off the planned
paths and 0.07 deg off target, vertex-based object clearance 11.0 mm (same as
the original plans).

With photo stops (8 per cycle: the sweep views on the pass from A to B, the arc
views on the way down), add `stops`:

```bash
python3 tools/make_combo.py outputs/scan_4_positions.npz outputs/scan_upward_arc.npz outputs/combo_stops.npz 1.0 stops
```

The stops are planned at 1 s (`dwell_seconds`) and land on the planned photo
poses; follow_trajectory.py `--hold` rescales them. 23.3 s cycle at 1 s stops.
