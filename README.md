# TRLC-DK1 scanning motion demo

```bash
git clone --recurse-submodules git@github.com:GrzegorzCzput/robot_demonstracja_skanowanie.git
# already cloned without it?  git submodule update --init
```

The official robot model and driver are in the `third_party/trlc-dk1` submodule,
pinned to upstream commit `12f5368`.

## Run on the real robot

Needs Python 3.12+ and the arm on USB. The saved trajectories in `outputs/` are
played as they are; no GPU or cuRobo is needed. The first run creates a local
virtual environment in `.venv-robot/` (numpy, pyserial, mujoco from
`requirements-robot.txt`); nothing is installed system-wide. Your user needs
access to the serial port (`sudo usermod -aG dialout $USER`, then log in again).

```bash
./run_robot.sh --dry-run                                   # no robot: checks and timing only
./run_robot.sh --port /dev/ttyACM0 --speed 0.3 --loops 1   # first real test: slow, one pass
./run_robot.sh --port /dev/ttyACM0                         # loop the four-view scan
./run_robot.sh --port /dev/ttyACM0 --mode upward_arc       # one upward pass, stops at the top
```

`./run_robot_docker.sh` takes the same arguments and runs everything in Docker
instead. The arm moves slowly to the first pose after
you press Enter, then plays the trajectory, aborting if any joint is more than
0.25 rad from the plan.

| Key | Effect |
|---|---|
| **Esc** | Return slowly to the startup pose, then disable the motors (no prompt) |
| Ctrl+C | Return slowly to the startup pose, then ask before disabling the motors |
| **Robot power switch** | Emergency stop. Both keys are software and drive home first |

Safety notes:

- Disabling the motors makes the arm go limp. Start with the arm in a low,
  clear pose so it does not fall far.
- The moves to the first pose and back home are straight joint-space
  interpolations and are **not collision checked**.
- The planning model does not include the phone or its mount; minimum planned
  clearance is 11.6 mm. Do first runs without the object and at `--speed 0.3`.
- The driver opens the gripper at startup to calibrate it.
- Arrow keys also trigger Esc.

## Simulation viewer

Open **http://127.0.0.1:8080** and choose **Motion mode**:

- **Four-view scan:** a 130° sweep with flatter outer camera views, followed by
  a return along the same arc. Saved as `outputs/scan_4_positions.npz`.
- **Upward arc:** one ascending arc in a vertical plane, always **38 cm from
  the object center**, starting at 25° and rising as high as the robot can
  continuously reach on that arc. The current top is about **74.4° elevation**
  and **46.6 cm Z** in the robot base frame. The camera remains level and
  stops at the top. Looping is disabled so it never automatically travels down.
  Saved as `outputs/scan_upward_arc.npz`.

Both modes take **18 seconds**, including **four 1-second holds**. Motion takes
14 seconds. Switching mode starts the selected simulation from its initial pose.
The viewer also has play/pause, restart, time scrubbing, 0.1–1× speed, and
scene/robot-camera viewpoints. Captures are simulated holds; no images are acquired.

## Run

The project has its own `.venv`. Planning uses the working local cuRobo 0.8
checkout at `/home/jan/Documents/grip/curobo`, PyTorch 2.9.1 + CUDA 13.0, and the
RTX 5070 Ti Laptop GPU. This checkout exposes `MotionPlanner`, the newer API
corresponding to the older cuRobo `MotionGen` API. It is a local research fork;
`requirements.lock` records the installed environment.

```bash
# Already installed; rebuild only if needed. Set CUROBO_SOURCE for another checkout.
./setup.sh

# Generate both trajectories on the GPU; errors abort instead of saving a failed plan.
.venv/bin/python plan_scan.py --mode all

# Replay an existing NPZ; no GPU is needed for this step.
.venv/bin/python view_scan.py

# Start directly in the upward mode.
.venv/bin/python view_scan.py --mode upward_arc

# Verify the saved trajectory and export a plot.
.venv/bin/python -m unittest test_trajectory -v
.venv/bin/python plot_trajectory.py --mode all
```

## Motion and limits

`config/demo.yaml` controls object position, four observation angles, camera
roll, per-view radii and heights, dwell, total duration, and joint limit scaling. The original
wide views are retained at **115°, 150°, 210°, and 245°**, spanning **130°**,
with gravity-referenced camera rolls **−15°, 0°, 0°, and +15°**. The optical
image-down axis is always the same physical axis, camera **−Z**. The middle
views remain at their previous positions and are upright. To level views 1 and
4 within the wrist limits and keep the gripper clear of the object, their
horizontal radii were reduced from 28 to **23 cm** and their heights from
34 to **28 cm**. This changes their camera positions by about 8 cm and reduces
their roll from ±60° to **±15°**. Views 2 and 3 retain a 28 cm radius and 34 cm
height. `upward_arc` separately configures the fixed distance, azimuth, starting
elevation, reachability-search ceiling, endpoint margin, and guide resolution.
The planner searches upward with collision-aware cuRobo IK, applies tight FK
pose checks, and bisects the first feasibility boundary to 0.01°. It then
subtracts a 0.5° margin and distributes four photo views from the starting
elevation to that top. This finds the highest continuously reachable pose on
the specified constant-distance arc, rather than maximizing height over
arbitrary camera distances. The found limit, first failed elevation, margin,
and maximum camera Z are recorded in `outputs/validation_upward_arc.json`.
Rebuild the robot model with `build_robot.py`
after changing the robot configuration or its acceleration reference.
Regenerate the trajectory after changing any planning configuration.

The official robot URDF specifies **5.4 rad/s for all six arm joints**. The speed
caps were raised at the user's request to fit the 18-second cycle: the velocity
cap is now **2.7 rad/s**, or 50% of the URDF rating. Its numerical limit tags, including
the more restrictive joint-six position limits, are used rather than the
different motor speeds and ranges in the URDF comments.

**Manufacturer maximum joint accelerations are not published in the URDF or
upstream controller configuration.** The current simulation explicitly assumes
a reference of **6.5 rad/s²** per joint with a factor of 1.0, giving a chosen
cap of **6.5 rad/s²**. This was increased slightly to retain the original wide views and gravity-guided return in
the requested 18-second loop. It is an assumed simulation limit, not a verified
manufacturer acceleration limit. The prior 10% caps are superseded by the
requested faster timing; physical joint-angle limits remain unchanged.
Replace `acceleration_reference_rad_s2` and `acceleration_reference_source` with
the actual values before treating it as such, then rebuild and replan.

cuRobo solves collision-aware IK at one-degree intervals along the viewing arc.
Each solve is seeded from its neighbor, and the nearest feasible solution is
selected to avoid joint-branch jumps. PCHIP profiles smoothly vary radius,
height, and roll between scan views. Its return retraces the viewing arc rather
than taking a joint-space shortcut that can turn the image upside down. The
upward mode instead follows a constant-radius elevation arc and has no return.
No extra holds are
introduced at the guide samples. A C2 cubic path in joint arc length interpolates
the cuRobo samples; a septic
time blend makes velocity, acceleration, and jerk settle to zero at move
boundaries. Each spline interval is composed with that clock; Bernstein
polynomial convex-hull bounds constrain velocity and acceleration throughout
the continuous curve. Remaining time is distributed among the four moves on
the 20 ms sample grid, so the full loop is exactly 18 seconds. A requested
duration that cannot meet the configured limits raises an error. Pauses have
exactly zero velocity and acceleration.

`outputs/validation.json` records measured peaks, view aiming errors, loop
closure, camera roll, image-down alignment, continuous-path aiming error, and
minimum sampled collision clearance. Every saved sample must keep the same
image-down axis downward, remain within ±20° roll in scan mode (±5° in upward
mode), and aim within 0.5° of the
object center. The resulting maximum aiming error is recorded rather than
assuming that waypoint checks suffice. Viser highlights image down in purple
and displays the measured roll; the scanned-object box remains hidden.
The upward trajectory additionally checks monotonic camera height and
constant 38 cm distance; its measured maximum radius error is recorded in
`outputs/validation_upward_arc.json`. `outputs/joint_trajectory.png` and
`outputs/upward_arc_joint_trajectory.png` plot the two modes. Seven tests check endpoint
smoothness, uniform and arc-length retiming, continuous derivative bounds,
fixed-duration feasibility, and the actual saved loop including timing, aiming,
image-down alignment, roll, limits, and the ascending arc's radius, height,
terminal stop, and disabled-loop metadata.

## Simulation geometry

The official robot model and meshes come from
[robot-learning-co/trlc-dk1](https://github.com/robot-learning-co/trlc-dk1), pinned
to `12f5368aefd0381461f2c7ffbb5611b4e8c90de9`. Upstream licenses remain in
`third_party/trlc-dk1`.

The demo assumes a 20 cm cube at `[0.48, 0, 0.10]` m. Planning uses the original
URDF `camera` frame on `link6-7`, mounted at `[0.0471, 0, 0.063314]` m with
RPY `[0, 0.2618, 0]` radians (15 degrees downward). Its **+X** axis looks forward,
**+Z** points up, and **+Y** points left. Viser's camera frustum is explicitly
converted to its +Z-forward convention. No phone or additional mount is modeled.
The displayed 60-degree camera field of view is illustrative; intrinsics have
not been calibrated.

The world includes a tabletop at Z = −0.03 m. Robot and object coordinates use
Z = 0 as their mounting reference; the 30 mm tabletop offset is a simulation
assumption and is not a calibrated physical workcell.

Robot collision meshes are approximated with fitted spheres.
All saved samples are independently checked against
the table, object, and enabled sphere pairs after smoothing. Immediate assembly
neighbors and listed shoulder/wrist housing neighbors are excluded in
`config/trlc_dk1.yaml`; the latter avoid sphere protrusion around shared joint
housings. This is sampled collision checking with approximate geometry, not a
continuous collision proof or hardware execution validation. No robot commands
are sent by this project.

## NPZ format

Load with `numpy.load(path, allow_pickle=False)`. All joints are ordered by
`joint_names` (`joint1` through `joint6`); angles are radians and times seconds.

| Key | Shape / content |
| --- | --- |
| `time` | `(N,)`, uniform 0.02 s samples, including the final endpoint |
| `position`, `velocity`, `acceleration` | `(N, 6)`, radians and their time derivatives |
| `phase` | `(N,)`, dwell or move label |
| `view_indices`, `view_joint_positions` | Four dwell-start indices and joint configurations |
| `view_positions`, `view_quaternions_wxyz` | Four requested robot-camera poses (+X optical) |
| `camera_position`, `camera_quaternion_wxyz` | Actual FK camera pose at every sample (+X optical) |
| `camera_roll_deg`, `image_down_world` | Gravity-referenced signed picture roll and the same image-down axis in world coordinates |
| `velocity_limits`, `acceleration_limits` | Per-joint scaled caps |
| `robot_max_velocity`, `acceleration_reference` | Original references, with acceleration explicitly assumed |
| `object_center` | Camera aiming target in the robot base frame |
| `metadata_json` | JSON validation report; includes the acceleration source |

The scan's final configuration equals its first. The upward arc's final
configuration is at the top and differs from its first; `loop_allowed` is false.

## API references

- Installed cuRobo 0.8 tutorial: `/home/jan/Documents/grip/curobo/curobo/examples/getting_started/motion_planning.py`
- [cuRobo legacy motion generation and time dilation](https://curobo.org/_api/curobo.wrap.reacher.motion_gen.html)
- [Viser trajectory line visualization](https://viser.studio/main/examples/scene/lines/)
# robot_demonstracja_skanowanie
