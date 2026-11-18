"""IK and approximate collision checks on the TRLC-DK1 URDF (no cuRobo).

Camera goal convention copied from Jan's plan_scan.goal_for: optical axis +X
points at the target, Y = world-up x X, then roll about X.
"""
import sys
from itertools import combinations
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent))  # fk.py = copy of render_upward_arc.py
from fk import Robot  # noqa: E402

URDF = Path(__file__).resolve().parent.parent / "third_party/trlc-dk1/urdf/follower/TRLC-DK1-Follower.urdf"
ROBOT = Robot(URDF)
CENTER = np.array([0.48, 0.0, 0.10])
HALF = np.full(3, 0.10)
TABLE_TOP = -0.03
# Driver limits minus its 0.05 rad clamp buffer and a little extra margin.
LIMITS = np.array([[-2.0944, 2.0944], [-0.0873, 3.1416], [-0.0873, 4.7124],
                   [-1.95, 1.5708], [-1.5708, 1.5708], [-2.0944, 2.0944]])
LO, HI = LIMITS[:, 0] + 0.055, LIMITS[:, 1] - 0.055

# Jan's self-collision exclusions (adjacent links and shared housings).
IGNORE = {frozenset(p) for p in [
    ("base_link", "link1-2"), ("base_link", "link2-3"), ("link1-2", "link2-3"), ("link2-3", "link3-4"),
    ("link3-4", "link4-5"), ("link3-4", "link5-6"), ("link4-5", "link5-6"), ("link4-5", "link6-7"),
    ("link5-6", "link6-7"), ("link5-6", "finger_left"), ("link5-6", "finger_right"),
    ("link6-7", "finger_left"), ("link6-7", "finger_right"), ("finger_left", "finger_right")]}
PAIRS = [p for p in combinations(ROBOT.meshes, 2) if frozenset(p) not in IGNORE]
VERTS = {k: np.unique(v.reshape(-1, 3), axis=0) for k, v in ROBOT.meshes.items()}


def goal_rotation(position, roll_deg=0.0):
    x = CENTER - position
    x /= np.linalg.norm(x)
    y = np.cross([0., 0., 1.], x)
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    return np.column_stack([x, y, z]) @ Rotation.from_euler("x", roll_deg, degrees=True).as_matrix()


def camera(q):
    return ROBOT.fk(q)["camera"]


def ik(position, roll_deg, seed):
    R_goal = goal_rotation(position, roll_deg)

    def residual(q):
        m = camera(q)
        rot = Rotation.from_matrix(R_goal.T @ m[:3, :3]).as_rotvec()
        return np.r_[m[:3, 3] - position, 0.1 * rot]

    sol = least_squares(residual, np.clip(seed, LO, HI), bounds=(LO, HI), xtol=1e-12, ftol=1e-12, gtol=1e-12)
    m = camera(sol.x)
    pos_err = np.linalg.norm(m[:3, 3] - position)
    rot_err = np.degrees(Rotation.from_matrix(R_goal.T @ m[:3, :3]).magnitude())
    return sol.x, pos_err, rot_err


def world_points(q):
    poses = ROBOT.fk(q)
    return {k: v @ poses[k][:3, :3].T + poses[k][:3, 3] for k, v in VERTS.items()}


def clearances(q):
    """(object, table, self) clearance in metres, vertex based (approximate)."""
    pts = world_points(q)
    arm = np.vstack([v for k, v in pts.items() if k != "base_link"])
    d = np.abs(arm - CENTER) - HALF
    obj = (np.linalg.norm(np.clip(d, 0, None), axis=1) + np.minimum(d.max(1), 0)).min()
    table = np.vstack([v for k, v in pts.items() if k not in ("base_link", "link1-2")])[:, 2].min() - TABLE_TOP
    self_c = min(cKDTree(pts[a]).query(pts[b], k=1)[0].min() for a, b in PAIRS)
    return obj, table, self_c


def roll_of(q):
    """Gravity-referenced picture roll in degrees (same definition as Jan's)."""
    r = camera(q)[:3, :3]
    up = np.array([0., 0., 1.]) - r[:, 0] * r[2, 0]
    up /= np.linalg.norm(up)
    return np.degrees(np.arctan2(r[:, 1] @ up, r[:, 2] @ up))
