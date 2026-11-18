"""Play a planned trajectory (upward arc, sweep or both combined) on a real TRLC-DK1 arm over USB.

Usually started through ./run_robot_docker.sh, which passes these options on.

Sequence: connect -> slow joint-space move to the first trajectory sample ->
play up and back down, repeating -> Esc or Ctrl+C -> slow move back to the pose
the arm had at startup -> disconnect.

Esc: return to the startup pose, then disable the motors without asking.
Ctrl+C: return to the startup pose, then ask before disabling the motors.

The approach and return moves are straight joint-space interpolations and are
NOT collision checked. Start and finish the arm in a clear pose above the table.
"""
import argparse
import json
import select
import sys
import termios
import threading
import tty
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
FILES = {"upward_arc": "trajectory/upward_arc.npz", "sweep": "trajectory/sweep.npz",
         "combo": "trajectory/combo.npz"}
# The upward arc moves slowly at 1x; 3.5x stays under its 3.73x cap
# (about 85% of the assumed 6.5 rad/s^2 acceleration cap).
# Sweep and combo are already timed at about 95% of the caps (constant speed, no stops).
DEFAULT_SPEED = {"upward_arc": 3.5, "sweep": 1.0, "combo": 1.0}
JOINTS = [f"joint{i}" for i in range(1, 7)]
# Same as trlc_dk1_control config.joint_pos_limits / URDF; the driver silently
# clips 0.05 rad inside these, so refuse trajectories that would be clipped.
LIMITS = np.array([[-2.0944, 2.0944], [-0.0873, 3.1416], [-0.0873, 4.7124],
                   [-1.95, 1.5708], [-1.5708, 1.5708], [-2.0944, 2.0944]])
LIMIT_BUFFER = 0.05


def load(path):
    with np.load(path, allow_pickle=False) as d:
        names = [str(n) for n in d["joint_names"]]
        if names != JOINTS:
            raise ValueError(f"Unexpected joint order {names}")
        t, q, qd = d["time"].astype(float), d["position"].astype(float), d["velocity"].astype(float)
        qdd = d["acceleration"].astype(float)
        vl, al = d["velocity_limits"].astype(float), d["acceleration_limits"].astype(float)
        loop = bool(np.array_equal(q[0], q[-1]))
        dwell = float(json.loads(str(d["metadata_json"]))["dwell_seconds"])
    if np.any(q < LIMITS[:, 0] + LIMIT_BUFFER) or np.any(q > LIMITS[:, 1] - LIMIT_BUFFER):
        raise ValueError("Trajectory enters the driver's joint-limit buffer")
    if np.any(np.diff(t) <= 0):
        raise ValueError("Trajectory time is not strictly increasing")
    # Scaling time by s scales velocity by s and acceleration by s**2.
    with np.errstate(divide="ignore"):
        max_speed = min(np.min(vl / np.abs(qd).max(0)), np.sqrt(np.min(al / np.abs(qdd).max(0))))
    return t, q, qd, loop, float(max_speed), dwell


def stationary_run(q):
    """Number of identical samples at the start of q (a hold)."""
    moving = np.flatnonzero(np.any(q[1:] != q[0], axis=1))
    return int(moving[0]) + 1 if len(moving) else len(q)


def bounce(q, qd):
    """Open path -> closed cycle: forward, then the same path back in reverse.

    Retracing the planned samples keeps the planner's collision, limit and
    aiming checks valid. The duplicate holds at the turnarounds are removed so
    the top and bottom stops stay one hold long.
    """
    back, back_v = q[::-1], -qd[::-1]
    top, bottom = stationary_run(back), stationary_run(q)
    back, back_v = back[top:len(back) - bottom], back_v[top:len(back) - bottom]
    return np.vstack([q, back, q[:1]]), np.vstack([qd, back_v, np.zeros((1, 6))])


def warp_time(q, qd, dt, speed, hold_scale=1.0):
    """Play moves `speed` times faster; holds last `hold_scale` times their planned length."""
    hold = np.all(q[1:] == q[:-1], axis=1)
    t = np.r_[0., np.cumsum(np.where(hold, dt * hold_scale, dt / speed))]
    return t, qd * speed


class EscWatcher:
    """Set `event` when Esc is pressed. Ctrl+C keeps working (cbreak mode)."""

    def __init__(self, event):
        self.event, self.running = event, False

    def __enter__(self):
        if not sys.stdin.isatty():
            print("stdin is not a terminal: Esc is unavailable, use Ctrl+C", flush=True)
            return self
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self.running = True
        self.thread = threading.Thread(target=self._watch, daemon=True)
        self.thread.start()
        return self

    def _watch(self):
        while self.running:
            if select.select([sys.stdin], [], [], 0.05)[0] and sys.stdin.read(1) == "\x1b":
                self.event.set()

    def __exit__(self, *exc):
        if self.running:
            self.running = False
            self.thread.join()
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
            termios.tcflush(self.fd, termios.TCIFLUSH)


class FakeRobot:
    """Stand-in for DK1Robot: the measured position equals the last command."""

    def __init__(self, q):
        self.q = np.asarray(q, float).copy()

    def connect(self): pass
    def disconnect(self): pass
    def get_joint_state(self): return {"pos": self.q.copy()}
    def command_joint_pos(self, q): self.q = q.copy()
    def set_velocity_feedforward(self, v): pass


def connect_robot(port):
    # Official driver, installed in the Docker image (see Dockerfile).
    from trlc_dk1_control import DK1Robot, DK1_DEFAULT_CONFIG
    robot = DK1Robot(DK1_DEFAULT_CONFIG(port))

    # DK1Robot always sends a zero velocity target, so its damping term (kd)
    # brakes every motion and the arm lags the plan. Inject the planned joint
    # velocity instead. It is dropped while the driver is in damping mode
    # (kp == 0) and whenever this program stops sending commands.
    lock, ff = threading.Lock(), {"v": np.zeros(6), "t": 0.}
    chain = robot._motor_chain
    send = chain.set_arm_commands

    def set_arm_commands(kp, kd, q, dq, tau):
        with lock:
            fresh = time.monotonic() - ff["t"] < 0.1
            v = ff["v"].copy()
        send(kp, kd, q, v if fresh and np.any(kp > 0) else dq, tau)

    def set_velocity_feedforward(v):
        with lock:
            ff["v"], ff["t"] = np.asarray(v, float).copy(), time.monotonic()

    chain.set_arm_commands = set_arm_commands
    robot.set_velocity_feedforward = set_velocity_feedforward
    robot.connect()
    return robot


def move_to(robot, target, max_speed, rate, stop=None):
    """Smooth (zero velocity/acceleration at both ends) joint-space move."""
    start = robot.get_joint_state()["pos"]
    distance = np.max(np.abs(target - start))
    duration = max(1.0, 1.875 * distance / max_speed)  # quintic peak speed = 1.875 * d / T
    t0 = time.monotonic()
    while (s := (time.monotonic() - t0) / duration) < 1:
        if stop is not None and stop.is_set():
            return False
        blend = s**3 * (10 - 15 * s + 6 * s**2)
        robot.command_joint_pos(start + blend * (target - start))
        robot.set_velocity_feedforward(np.zeros(6))
        time.sleep(1 / rate)
    robot.command_joint_pos(target.copy())
    return True


def play(robot, t, q, qd, loop, rate, max_error, stop):
    """Stream the trajectory against the wall clock until done or stopped."""
    t0, cycle = time.monotonic(), 0
    while not stop.is_set():
        elapsed = time.monotonic() - t0
        if elapsed >= t[-1]:
            cycle += 1
            print(f"Cycle {cycle} complete", flush=True)
            if not loop:
                break
            t0 += t[-1]
            elapsed -= t[-1]
        q_cmd = np.array([np.interp(elapsed, t, q[:, j]) for j in range(6)])
        v_cmd = np.array([np.interp(elapsed, t, qd[:, j]) for j in range(6)])
        robot.command_joint_pos(q_cmd)
        robot.set_velocity_feedforward(v_cmd)
        error = np.abs(robot.get_joint_state()["pos"] - q_cmd)
        if error.max() > max_error:
            print(f"ABORT: joint{error.argmax() + 1} is {error.max():.3f} rad from the plan "
                  f"(limit {max_error} rad). Holding position.", flush=True)
            robot.command_joint_pos(robot.get_joint_state()["pos"])
            return False
        time.sleep(1 / rate)
    robot.command_joint_pos(robot.get_joint_state()["pos"])  # hold where we are
    return True


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0", help="Robot USB serial port")
    p.add_argument("--mode", choices=tuple(FILES), default="upward_arc")
    p.add_argument("--trajectory", type=Path, help="Override the NPZ file for --mode")
    p.add_argument("--speed", type=float,
                   help="Motion speed factor, 0.1 up to the trajectory's limit; holds stay 1 s "
                        "(default: 3.5 upward arc, 1.0 sweep and combo)")
    p.add_argument("--hold", type=float, default=0.5,
                   help="Length of each photo stop in seconds (default: 0.5)")
    p.add_argument("--stops", action="store_true",
                   help="Combo: stop at the 8 photo positions (length set by --hold)")
    p.add_argument("--once", action="store_true",
                   help="Single pass, stop at the top (default: go back down and repeat)")
    p.add_argument("--loops", type=int, default=0, help="Stop after N cycles (0 = until Ctrl+C)")
    p.add_argument("--rate", type=float, default=100., help="Command rate in Hz")
    p.add_argument("--approach-speed", type=float, default=0.3, help="Peak rad/s for approach/return")
    p.add_argument("--max-error", type=float, default=0.25, help="Abort if tracking error exceeds this (rad)")
    p.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    p.add_argument("--dry-run", action="store_true", help="No hardware; simulate the robot")
    a = p.parse_args()
    if a.stops and a.mode != "combo":
        p.error("--stops is only for --mode combo (the upward arc always stops; the sweep never does)")
    path = a.trajectory or ROOT / ("trajectory/combo_stops.npz" if a.stops else FILES[a.mode])
    if a.speed is None:
        a.speed = DEFAULT_SPEED[a.mode]
    t, q, qd, loop, max_speed, dwell = load(path)
    if not 0 <= a.hold <= 10:
        p.error("--hold must be between 0 and 10 seconds")
    max_speed = np.floor(max_speed * 100) / 100
    if not 0.1 <= a.speed <= max_speed:
        p.error(f"--speed must be between 0.1 and {max_speed:.2f} for {path.name} "
                "(its velocity and acceleration caps)")
    if not loop and not a.once:
        q, qd = bounce(q, qd)
        loop = True
    t, qd = warp_time(q, qd, t[1] - t[0], a.speed, a.hold / dwell if dwell else 1.0)
    if a.loops:
        # Repeat the cycle N times and play it once.
        n = a.loops if loop else 1
        t = np.concatenate([t[:-1] + i * t[-1] for i in range(n)] + [[n * t[-1]]])
        q = np.vstack([q[:-1]] * n + [q[-1:]])
        qd = np.vstack([qd[:-1]] * n + [qd[-1:]])
        loop = False
    print(f"{path.name}: {'looping' if loop else 'single pass'}, speed {a.speed:.2f}x "
          f"(max {max_speed:.2f}x), {f'{a.hold:g} s stops' if dwell else 'no stops'}, cycle {t[-1]:.1f} s", flush=True)

    robot = FakeRobot([0., 1.4, 2.3, 0.3, 0., 0.]) if a.dry_run else connect_robot(a.port)
    stop = threading.Event()
    try:
        home = robot.get_joint_state()["pos"]
        print(f"Current joints: {np.round(home, 3).tolist()}", flush=True)
        print(f"First pose:     {np.round(q[0], 3).tolist()}", flush=True)
        if not (a.yes or a.dry_run):
            input("Keep clear of the arm. Press Enter to move to the first pose (Ctrl+C aborts)... ")
        try:
            with EscWatcher(stop):
                print("Moving to the first pose. Esc = go home and disable, Ctrl+C = go home.", flush=True)
                if move_to(robot, q[0], a.approach_speed, a.rate, stop):
                    print("Playing. Esc = go home and disable, Ctrl+C = go home.", flush=True)
                    play(robot, t, q, qd, loop, a.rate, a.max_error, stop)
            if stop.is_set():
                robot.command_joint_pos(robot.get_joint_state()["pos"])
                print("Esc pressed.", flush=True)
        except KeyboardInterrupt:
            robot.command_joint_pos(robot.get_joint_state()["pos"])
            print("\nStopped.", flush=True)
        print("Returning to the startup pose (Ctrl+C to skip and hold)...", flush=True)
        try:
            move_to(robot, home, a.approach_speed, a.rate)
        except KeyboardInterrupt:
            print("\nReturn skipped.", flush=True)
    except KeyboardInterrupt:
        print("\nAborted.", flush=True)
    finally:
        if stop.is_set():
            print("Disabling motors.", flush=True)
        elif not a.dry_run:
            input("Motors will be DISABLED and the arm will go limp. Support it, then press Enter... ")
        robot.disconnect()


if __name__ == "__main__":
    main()
