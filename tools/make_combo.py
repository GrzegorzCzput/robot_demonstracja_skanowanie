"""Combined cycle: side-to-side sweep, then up-and-down arc, joined in the middle.

    python3 make_combo.py scan_4_positions.npz upward_arc.npz combo.npz

Cycle (starts and ends at the sweep's middle pose M, at rest):
  sweep  M -> end A -> end B -> M          constant speed, reversals at the ends
  switch M -> P (arc at ~40 deg)           short joint-space move, rest to rest
  arc    P -> top -> bottom -> P           constant speed, reversals at the ends
  switch P -> M
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


class JointLeg:
    """Rest-to-rest quintic joint move (the short switch between the two paths)."""
    def __init__(self, qa, qb):
        self.qa, self.dq = qa, qb - qa
        d = np.abs(self.dq)
        self.duration = max(0.2, np.max(np.maximum(1.875 * d / VL, np.sqrt(5.7735 * d / AL))))

    def at(self, t):
        u = min(t / self.duration, 1.)
        T = self.duration
        b = u**3 * (10 - 15 * u + 6 * u**2)
        b1 = 30 * u**2 * (1 - u)**2
        b2 = 60 * u * (1 - u) * (1 - 2 * u)
        return self.qa + self.dq * b, self.dq * b1 / T, self.dq * b2 / T**2


def build(a_sweep, a_arc):
    vs, va = cruise(sweep), cruise(arc)
    L, top = sweep.x[-1], arc.x[-1]
    return [
        PathLeg(sweep, s_mid, 0., "rest", "rev", vs, a_sweep),
        PathLeg(sweep, 0., L, "rev", "rev", vs, a_sweep),
        PathLeg(sweep, L, s_mid, "rev", "rest", vs, a_sweep),
        JointLeg(q_M, q_P),
        PathLeg(arc, s_P, top, "rest", "rev", va, a_arc),
        PathLeg(arc, top, 0., "rev", "rev", va, a_arc),
        PathLeg(arc, 0., s_P, "rev", "rest", va, a_arc),
        JointLeg(q_P, q_M),
    ]


def sample(legs):
    total = sum(l.duration for l in legs)
    t = np.r_[np.arange(0, total, DT), total]
    starts = np.cumsum([0] + [l.duration for l in legs])
    Q, V, A = [], [], []
    for ti in t:
        i = min(np.searchsorted(starts, ti, side="right") - 1, len(legs) - 1)
        q, v, a = legs[i].at(ti - starts[i])
        Q.append(q); V.append(v); A.append(a)
    Q, V, A = map(np.asarray, (Q, V, A))
    Q[-1], V[-1], A[-1] = Q[0], 0., 0.
    return t, Q, V, A


def peak_acc(sp, a):
    """Worst joint acceleration / cap for path acceleration a at cruise on this path."""
    legs = [PathLeg(sp, 0., sp.x[-1], "rev", "rev", cruise(sp), a)]
    _, _, _, A = sample(legs)
    return np.max(np.abs(A) / AL)


def largest_a(sp):
    lo, hi = 0.1, 100.
    for _ in range(40):
        mid = np.sqrt(lo * hi)
        lo, hi = (mid, hi) if peak_acc(sp, mid) <= 1 else (lo, mid)
    return lo


a_sweep, a_arc = largest_a(sweep), largest_a(arc)
legs = build(a_sweep, a_arc)
t, Q, V, A = sample(legs)
# Safety net: shrink both accelerations until the whole cycle is within the caps.
while np.max(np.abs(A) / AL) > 1 or np.max(np.abs(V) / VL) > 1:
    a_sweep, a_arc = 0.97 * a_sweep, 0.97 * a_arc
    legs = build(a_sweep, a_arc)
    t, Q, V, A = sample(legs)

names = ["sweep to end A", "sweep across", "sweep back to middle", "switch to arc",
         "arc up", "arc down", "arc back to 40 deg", "switch to sweep"]
report = dict(mode="combo", dwell_seconds=0.0, cycle_seconds=float(t[-1]),
              legs={n: round(l.duration, 3) for n, l in zip(names, legs)},
              junction_joint_gap_rad=float(np.abs(q_P - q_M).max()),
              sweep_path_accel=a_sweep, arc_path_accel=a_arc, margin_of_caps=MARGIN,
              peak_velocity_fraction=float(np.max(np.abs(V) / scan["velocity_limits"])),
              peak_acceleration_fraction=float(np.max(np.abs(A) / scan["acceleration_limits"])))
np.savez_compressed(OUT, time=t, position=Q, velocity=V, acceleration=A,
                    joint_names=scan["joint_names"], velocity_limits=scan["velocity_limits"],
                    acceleration_limits=scan["acceleration_limits"], object_center=scan["object_center"],
                    metadata_json=np.array(json.dumps(report)))
print(json.dumps(report, indent=1))
