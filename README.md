# TRLC-DK1 scanning demo

The arm moves a camera (phone) around an object, always pointing at its center.
There are three motions; **combo with photo stops is the default**.

- **Combo** (default): side to side, then up and down, repeating. It stops for
  0.5 s at 8 photo positions and curves from one motion into the other in the
  middle without stopping. One cycle takes 19.5 s. With `--no-stops` it moves
  continuously at constant speed instead (9.3 s per cycle).
- **Upward arc** (`--mode upward_arc`): up an arc 38 cm from the object, stopping
  for 0.5 s at four photo positions, then back down, repeating.
- **Sweep** (`--mode sweep`): side to side around the object (110°) at constant
  speed, with no stops. It only slows down to turn around at the two ends.
  One cycle takes 5.1 s. At the two ends the camera is 30 cm from the object.

![Robot at the four photo positions](docs/upward_arc.png)

## You need

- Linux with [Docker](https://docs.docker.com/engine/install/)
- The TRLC-DK1 arm, powered on and connected by USB

## Run it

```bash
git clone https://github.com/GrzegorzCzput/robot_demonstracja_skanowanie.git
cd robot_demonstracja_skanowanie

./run_robot_docker.sh --dry-run             # try it without the robot
./run_robot_docker.sh                       # run it on the robot (combo with photo stops)
./run_robot_docker.sh --no-stops            # combo, continuous
./run_robot_docker.sh --mode sweep          # side to side only
./run_robot_docker.sh --mode upward_arc     # up and down only
```

The first run builds the Docker image, which takes a minute or two. After that:

1. The script shows the arm's current position and waits. **Press Enter.**
2. The arm moves slowly to the start of the motion.
3. It repeats the motion until you stop it.

## Stop it

| Press | What happens |
|---|---|
| **Esc** | The arm goes back to where it started, then the motors turn off |
| **Ctrl+C** | Same, but asks before turning the motors off, so you can hold the arm |
| **Robot power switch** | Emergency stop |

When the motors turn off, the arm goes limp and drops.

## Options

| Option | Meaning |
|---|---|
| `--port /dev/ttyACM1` | The robot isn't on `/dev/ttyACM0`. Find it with `ls /dev/ttyACM*` |
| `--speed 1.5` | Slower or faster motion. Upward arc: default 3.5, max 3.73. Sweep and combo: default 1.0, max 1.02 |
| `--no-stops` | Combo: move continuously, without photo stops |
| `--hold 1` | Photo stop length in seconds (the default is 0.5) |
| `--loops 320` | Stop after 320 cycles (combo 19.5 s, arc 11 s, sweep 5.1 s each) |
| `--once` | Upward arc: go up once and stop at the top |

## Before the first run

- Clear the area around the robot. The move to the start of the arc and the move
  back home are not collision checked.
- Start with the arm in a low, safe pose. It returns there at the end.
- Try a slow speed first (`--speed 0.5`; for the upward arc `--speed 1.5`),
  without the object on the table.
- Keep a hand near the power switch.

The upward arc was planned with collision checking in simulation (cuRobo). The
side-to-side path was then re-solved from the robot model, with its ends moved
out to 30 cm, and checked for joint limits, camera aim and approximate mesh
clearance (at least 70 mm from the object, 22 mm between robot parts). Where
the combo curves from one motion into the other, the camera is at most 22 mm
off the planned paths, more than 11 cm from the object. The original planner,
simulation viewer, the generators for these trajectories and the four-view scan are on the
[`full-project`](https://github.com/GrzegorzCzput/robot_demonstracja_skanowanie/tree/full-project)
branch.
