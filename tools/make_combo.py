"""Combined cycle: side-to-side sweep, then up-and-down arc, joined in the middle.

    python3 make_combo.py scan_4_positions.npz upward_arc.npz combo.npz

Cycle (starts and ends at sweep end A, where the motion reverses anyway):
  sweep  A -> B -> middle M     constant speed, reversals at the ends
  switch M -> arc at ~40 deg    without stopping: the sweep joints (1, 5, 6) brake
                                while the arc joints (2, 3, 4) speed up
  arc    40 deg -> top -> bottom -> 40 deg
  switch arc -> M -> A          the same overlap in reverse
"""
import json
import sys

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import brentq

SCAN, ARC, OUT = sys.argv[1:4]
MARGIN, THIN, DT = 0.95, 0.03, 0.02

scan, arc_src = np.load(SCAN), np.load(ARC)
VL = scan["velocity_limits"] * MARGIN
AL = scan["acceleration_limits"] * MARGIN


def path_spline(q):
    """Spline through planned samples by joint-space arc length; thinning removes the
    rounding kinks left at the original stop points."""
    keep = [0]
    for i in range(1, len(q) - 1):
        if np.linalg.norm(q[i] - q[keep[-1]]) > THIN:
            keep.append(i)
    keep.append(len(q) - 1)
    p = q[keep]
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    return CubicSpline(s, p, bc_type="natural")


vi = scan["view_indices"]
sweep = path_spline(scan["position"][vi[0]:vi[3] + 1])
aq = arc_src["position"]
arc = path_spline(aq)  # bottom (25 deg) -> top

# Junction: sweep middle (joint1 crosses 0) and the closest arc pose in joint space.
s_mid = brentq(lambda s: sweep(s)[0], 0.2 * sweep.x[-1], 0.8 * sweep.x[-1])
q_M = sweep(s_mid)
s_grid = np.linspace(0, arc.x[-1], 20001)
s_P = s_grid[np.argmin(np.abs(arc(s_grid) - q_M).max(1))]
q_P = arc(s_P)


def cruise(sp):
    s = np.linspace(0, sp.x[-1], 20001)
    q1, q2 = np.abs(sp(s, 1)).max(0), np.abs(sp(s, 2)).max(0)
    return min(np.min(VL / q1), np.sqrt(np.min(AL / np.maximum(q2, 1e-9))))


K = {"rev": 1.0, "rest": 0.75}  # ramp distance = K * v^2 / a


def ramp(kind, start, v, a, t):
    """Distance, speed, path acceleration t seconds into a ramp of the given kind."""
    if kind == "rev":
        w = a / v
        if start:
            return v / w * (1 - np.cos(w * t)), v * np.sin(w * t), v * w * np.cos(w * t)
        return v / w * np.sin(w * t), v * np.cos(w * t), -v * w * np.sin(w * t)
    T = 1.5 * v / a
    u = t / T
    if start:
        return v * T * (u**3 - u**4 / 2), v * (3 * u**2 - 2 * u**3), v / T * (6 * u - 6 * u**2)
    r = 1 - u
    return v * T / 2 - v * T * (r**3 - r**4 / 2), v * (3 * r**2 - 2 * r**3), -v / T * (6 * r - 6 * r**2)


def ramp_time(kind, v, a):
    return np.pi * v / (2 * a) if kind == "rev" else 1.5 * v / a


class PathLeg:
    def __init__(self, sp, s0, s1, k0, k1, v_c, a):
        self.sp, self.s0, self.sign = sp, s0, np.sign(s1 - s0)
        self.k0, self.k1, self.a = k0, k1, a
        length = abs(s1 - s0)
        self.v = min(v_c, np.sqrt(length * a / (K[k0] + K[k1])))
        self.T0, self.T1 = ramp_time(k0, self.v, a), ramp_time(k1, self.v, a)
        self.d0 = K[k0] * self.v**2 / a
        self.Tc = (length - self.d0 - K[k1] * self.v**2 / a) / self.v
        self.duration = self.T0 + max(self.Tc, 0) + self.T1

    def at(self, t):
        if t < self.T0:
            d, sd, sdd = ramp(self.k0, True, self.v, self.a, t)
        elif t < self.T0 + self.Tc:
            d, sd, sdd = self.d0 + self.v * (t - self.T0), self.v, 0.
        else:
            tau = min(t - self.T0 - max(self.Tc, 0), self.T1)
            dd, sd, sdd = ramp(self.k1, False, self.v, self.a, tau)
            d = self.d0 + self.v * max(self.Tc, 0) + dd
        s = self.s0 + self.sign * d
        sd, sdd = self.sign * sd, self.sign * sdd
        return self.sp(s), self.sp(s, 1) * sd, self.sp(s, 2) * sd**2 + self.sp(s, 1) * sdd



def quintic(u):
    return u**3 * (10 - 15 * u + 6 * u**2), 30 * u**2 * (1 - u)**2, 60 * u * (1 - u) * (1 - 2 * u)


def build(a_sweep, a_arc, overlap=1.0):
    """Legs with start times. The switch between paths overlaps the braking of one
    path with the speed-up of the other (they use different joints), and closes
    the small joint gap between them with a quintic over the same window."""
    vs, va = cruise(sweep), cruise(arc)
    L, top = sweep.x[-1], arc.x[-1]
    legs = [PathLeg(sweep, 0., L, "rev", "rev", vs, a_sweep),
            PathLeg(sweep, L, s_mid, "rev", "rest", vs, a_sweep),
            PathLeg(arc, s_P, top, "rest", "rev", va, a_arc),
            PathLeg(arc, top, 0., "rev", "rev", va, a_arc),
            PathLeg(arc, 0., s_P, "rev", "rest", va, a_arc),
            PathLeg(sweep, s_mid, 0., "rest", "rev", vs, a_sweep)]
    switches = {2: q_P - q_M, 5: q_M - q_P}  # leg index -> gap closed while it starts
    starts, gaps, t = [], [], 0.
    for i, leg in enumerate(legs):
        if i in switches:
            O = overlap * min(legs[i - 1].T1, leg.T0)
            t -= O
            gaps.append((t, max(O, 0.25), switches[i]))
        starts.append(t)
        t += leg.duration
    return legs, starts, gaps, t


def sample(plan):
    legs, starts, gaps, total = plan
    t = np.r_[np.arange(0, total, DT), total]
    q0 = legs[0].at(0)[0]
    Q = np.tile(q0, (len(t), 1)); V = np.zeros_like(Q); A = np.zeros_like(Q)
    for leg, st in zip(legs, starts):
        qs, qe = leg.at(0)[0], leg.at(leg.duration)[0]
        for k, ti in enumerate(t):
            if ti >= st + leg.duration:
                Q[k] += qe - qs
            elif ti > st:
                q, v, a = leg.at(ti - st)
                Q[k] += q - qs; V[k] += v; A[k] += a
    for st, dur, gap in gaps:
        u = np.clip((t - st) / dur, 0, 1)
        b, b1, b2 = quintic(u)
        inside = (t > st) & (t < st + dur)
        Q += np.outer(b, gap); V += np.outer(b1 * inside / dur, gap); A += np.outer(b2 * inside / dur**2, gap)
    Q[-1], V[-1], A[-1] = Q[0], 0., 0.
    return t, Q, V, A


def peak_acc(sp, a):
    leg = PathLeg(sp, 0., sp.x[-1], "rev", "rev", cruise(sp), a)
    t = np.r_[np.arange(0, leg.duration, DT), leg.duration]
    A = np.array([leg.at(x)[2] for x in t])
    return np.max(np.abs(A) / AL)


def largest_a(sp):
    lo, hi = 0.1, 100.
    for _ in range(40):
        mid = np.sqrt(lo * hi)
        lo, hi = (mid, hi) if peak_acc(sp, mid) <= 1 else (lo, mid)
    return lo


OVERLAP = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
a_sweep, a_arc = largest_a(sweep), largest_a(arc)
plan = build(a_sweep, a_arc, OVERLAP)
t, Q, V, A = sample(plan)
while np.max(np.abs(A) / AL) > 1 or np.max(np.abs(V) / VL) > 1:
    a_sweep, a_arc = 0.98 * a_sweep, 0.98 * a_arc
    plan = build(a_sweep, a_arc, OVERLAP)
    t, Q, V, A = sample(plan)

legs, starts, gaps, total = plan
names = ["sweep across", "sweep back to middle", "arc up", "arc down", "arc back to 40 deg", "sweep to end A"]
report = dict(mode="combo", dwell_seconds=0.0, cycle_seconds=float(t[-1]),
              leg_starts={n: round(s, 3) for n, s in zip(names, starts)},
              switch_windows=[(round(s, 3), round(d, 3)) for s, d, _ in gaps], overlap=OVERLAP,
              junction_joint_gap_rad=float(np.abs(q_P - q_M).max()),
              sweep_path_accel=a_sweep, arc_path_accel=a_arc, margin_of_caps=MARGIN,
              peak_velocity_fraction=float(np.max(np.abs(V) / scan["velocity_limits"])),
              peak_acceleration_fraction=float(np.max(np.abs(A) / scan["acceleration_limits"])))
np.savez_compressed(OUT, time=t, position=Q, velocity=V, acceleration=A,
                    joint_names=scan["joint_names"], velocity_limits=scan["velocity_limits"],
                    acceleration_limits=scan["acceleration_limits"], object_center=scan["object_center"],
                    metadata_json=np.array(json.dumps(report)))
print(json.dumps(report, indent=1))
