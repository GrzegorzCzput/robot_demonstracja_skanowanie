# TRLC-DK1 scanning demo

The arm moves a camera (phone) up an arc around an object, stopping for 1 s at
four photo positions, then comes back down and repeats. The camera always points
at the object center and stays 38 cm from it.

![Robot at the four photo positions](docs/upward_arc.png)

## You need

- Linux with [Docker](https://docs.docker.com/engine/install/)
- The TRLC-DK1 arm, powered on and connected by USB

## Run it

```bash
git clone https://github.com/GrzegorzCzput/robot_demonstracja_skanowanie.git
cd robot_demonstracja_skanowanie

./run_robot_docker.sh --dry-run           # try it without the robot
./run_robot_docker.sh --mode upward_arc   # run it on the robot
```

The first run builds the Docker image, which takes a minute or two. After that:

1. The script shows the arm's current position and waits. **Press Enter.**
2. The arm moves slowly to the start of the arc.
3. It runs the arc up and down until you stop it.

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
| `--speed 1.5` | Slower motion (the default is 3.5, the maximum is 3.73). Photo stops stay 1 s |
| `--once` | Go up once and stop at the top |

## Before the first run

- Clear the area around the robot. The move to the start of the arc and the move
  back home are not collision checked.
- Start with the arm in a low, safe pose. It returns there at the end.
- Try `--speed 1.5` first, without the object on the table.
- Keep a hand near the power switch.

The trajectory was planned with collision checking in simulation. The planner,
simulation viewer and the four-view scan are on the
[`full-project`](https://github.com/GrzegorzCzput/robot_demonstracja_skanowanie/tree/full-project)
branch.
