"""Play a planned NPZ trajectory on a real TRLC-DK1 arm connected over USB.

    python follow_trajectory.py --port /dev/ttyACM0             # four-view scan, loops until Ctrl+C
    python follow_trajectory.py --port /dev/ttyACM0 --speed 0.5 # half speed (recommended first run)
    python follow_trajectory.py --mode upward_arc               # one pass, stops at the top
    python follow_trajectory.py --dry-run                       # no hardware; checks and timing only

Sequence: connect -> slow joint-space move to the first trajectory sample ->
play (looping if the trajectory allows it) -> Esc or Ctrl+C -> slow move back
to the pose the arm had at startup -> disconnect.

Esc: return to the startup pose, then disable the motors without asking.
Ctrl+C: return to the startup pose, then ask before disabling the motors.

The approach and return moves are straight joint-space interpolations and are
NOT collision checked. Start and finish the arm in a clear pose above the table.
"""
import argparse
import select
import sys
import termios
import threading
import tty
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
FILES = {"scan": "scan_4_positions.npz", "upward_arc": "scan_upward_arc.npz"}
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
        loop = bool(np.array_equal(q[0], q[-1]))
    if np.any(q < LIMITS[:, 0] + LIMIT_BUFFER) or np.any(q > LIMITS[:, 1] - LIMIT_BUFFER):
        raise ValueError("Trajectory enters the driver's joint-limit buffer")
    if np.any(np.diff(t) <= 0):
        raise ValueError("Trajectory time is not strictly increasing")
    return t, q, qd, loop


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
    try:
        from trlc_dk1_control import DK1Robot, DK1_DEFAULT_CONFIG
    except ImportError:
        sys.path.insert(0, str(ROOT / "third_party/trlc-dk1"))
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


def play(robot, t, q, qd, loop, speed, rate, max_error, stop):
    """Stream the trajectory against the wall clock until done or stopped."""
    t0, cycle = time.monotonic(), 0
    while not stop.is_set():
        elapsed = (time.monotonic() - t0) * speed
        if elapsed >= t[-1]:
            cycle += 1
            print(f"Cycle {cycle} complete", flush=True)
            if not loop:
                break
            t0 += t[-1] / speed
            elapsed -= t[-1]
        q_cmd = np.array([np.interp(elapsed, t, q[:, j]) for j in range(6)])
        v_cmd = np.array([np.interp(elapsed, t, qd[:, j]) for j in range(6)]) * speed
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
    p.add_argument("--mode", choices=tuple(FILES), default="scan")
    p.add_argument("--trajectory", type=Path, help="Override the NPZ file for --mode")
    p.add_argument("--speed", type=float, default=1.0, help="Playback speed factor, 0.1-1.0")
    p.add_argument("--loops", type=int, default=0, help="Stop after N cycles (0 = until Ctrl+C)")
    p.add_argument("--rate", type=float, default=100., help="Command rate in Hz")
    p.add_argument("--approach-speed", type=float, default=0.3, help="Peak rad/s for approach/return")
    p.add_argument("--max-error", type=float, default=0.25, help="Abort if tracking error exceeds this (rad)")
    p.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    p.add_argument("--dry-run", action="store_true", help="No hardware; simulate the robot")
    a = p.parse_args()
    if not 0.1 <= a.speed <= 1.0:
        p.error("--speed must be between 0.1 and 1.0")

    path = a.trajectory or ROOT / "outputs" / FILES[a.mode]
    t, q, qd, loop = load(path)
    if a.loops:
        # Repeat the cycle N times and play it once.
        n = a.loops if loop else 1
        t = np.concatenate([t[:-1] + i * t[-1] for i in range(n)] + [[n * t[-1]]])
        q = np.vstack([q[:-1]] * n + [q[-1:]])
        qd = np.vstack([qd[:-1]] * n + [qd[-1:]])
        loop = False
    print(f"{path.name}: {t[-1]:.1f} s per pass at 1x, "
          f"{'looping' if loop else 'single pass'}, playing at {a.speed:.1f}x", flush=True)

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
                    play(robot, t, q, qd, loop, a.speed, a.rate, a.max_error, stop)
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
