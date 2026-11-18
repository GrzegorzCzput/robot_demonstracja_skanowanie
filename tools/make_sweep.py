"""Continuous side-to-side sweep from Jan's planned four-view scan path.

Constant speed along the planned path, sinusoidal braking/reversal at the two
ends only (acceleration continuous, velocity zero only at the reversal instant).
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline

src, out = Path(sys.argv[1]), Path(sys.argv[2])
MARGIN = 0.95  # use 95% of the caps
THIN = float(sys.argv[3]) if len(sys.argv) > 3 else 0.03  # rad; removes rounding kinks at the old stops
DT = 0.02

d = np.load(src, allow_pickle=False)
q, vi = d["position"], d["view_indices"]
vl, al = d["velocity_limits"] * MARGIN, d["acceleration_limits"] * MARGIN
fwd = q[vi[0]:vi[3] + 1]

# Thin out the clustered samples near each original stop; keep the endpoints.
keep = [0]
for i in range(1, len(fwd) - 1):
    if np.linalg.norm(fwd[i] - fwd[keep[-1]]) > THIN:
        keep.append(i)
keep.append(len(fwd) - 1)
P = fwd[keep]
s_knots = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
L = s_knots[-1]
path = CubicSpline(s_knots, P, bc_type="natural")
d1, d2 = path.derivative(1), path.derivative(2)

s_fine = np.linspace(0, L, 20001)
q1, q2 = np.abs(d1(s_fine)), np.abs(d2(s_fine))
# Cruise: velocity cap and centripetal acceleration cap everywhere on the path.
v_c = min(np.min(vl / q1.max(0)), np.sqrt(np.min(al / np.maximum(q2, 1e-9).max(0))))


def profile(T):
    """Return t, s, sd, sdd for one forward pass (rest -> L -> rest), reversal ramps of length T."""
    ramp = v_c * 2 * T / np.pi
    cruise = L - 2 * ramp
    if cruise < 0:
        return None
    Tc = cruise / v_c
    total = 2 * T + Tc
    n = int(np.ceil(total / DT))
    t = np.linspace(0, total, n + 1)
    w = np.pi / (2 * T)
    s, sd, sdd = np.empty_like(t), np.empty_like(t), np.empty_like(t)
    a = t <= T
    s[a], sd[a], sdd[a] = v_c / w * (1 - np.cos(w * t[a])), v_c * np.sin(w * t[a]), v_c * w * np.cos(w * t[a])
    b = (t > T) & (t < T + Tc)
    s[b], sd[b], sdd[b] = ramp + v_c * (t[b] - T), v_c, 0.
    c = t >= T + Tc
    tau = t[c] - T - Tc
    s[c], sd[c], sdd[c] = L - ramp + v_c / w * np.sin(w * tau), v_c * np.cos(w * tau), -v_c * w * np.sin(w * tau)
    s[-1], sd[-1] = L, 0.
    return t, s, sd, sdd


def joint(prof):
    t, s, sd, sdd = prof
    qq = path(s)
    qd = d1(s) * sd[:, None]
    qdd = d2(s) * sd[:, None] ** 2 + d1(s) * sdd[:, None]
    return t, qq, qd, qdd


# Shortest reversal ramp that keeps every joint under the acceleration cap.
T = None
for cand in np.arange(0.05, 5, 0.01):
    prof = profile(cand)
    if prof is None:
        break
    _, _, qd, qdd = joint(prof)
    if np.all(np.abs(qdd) <= al) and np.all(np.abs(qd) <= vl):
        T = cand
        break
if T is None:
    raise SystemExit("No feasible profile")

t, qq, qd, qdd = joint(profile(T))
# Full cycle: forward, then the same path back (time-reversed), closed loop.
back_q, back_v, back_a = qq[::-1][1:], -qd[::-1][1:], qdd[::-1][1:]
Q = np.vstack([qq, back_q])
V = np.vstack([qd, back_v])
A = np.vstack([qdd, back_a])
time = np.r_[t, t[-1] + (t[-1] - t[::-1][1:])]
Q[-1] = Q[0]
report = dict(mode="sweep", source="full-project:outputs/scan_4_positions.npz (forward path)",
              dwell_seconds=0.0, cruise_path_speed=float(v_c), reversal_ramp_s=float(T),
              cycle_seconds=float(time[-1]), margin_of_caps=MARGIN,
              peak_velocity_rad_s=np.abs(V).max(0).tolist(), peak_acceleration_rad_s2=np.abs(A).max(0).tolist())
np.savez_compressed(out, time=time, position=Q, velocity=V, acceleration=A,
                    joint_names=d["joint_names"], velocity_limits=d["velocity_limits"],
                    acceleration_limits=d["acceleration_limits"], object_center=d["object_center"],
                    metadata_json=np.array(json.dumps(report)))
print(json.dumps(report, indent=1))
print("path length", L, "knots", len(P))
