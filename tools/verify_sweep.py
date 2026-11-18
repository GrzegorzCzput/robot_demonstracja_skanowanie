import sys, json, numpy as np
sys.path.insert(0, sys.argv[3])
from fk import Robot
from pathlib import Path
robot = Robot(Path(sys.argv[4]))
orig = np.load(sys.argv[1]); new = np.load(sys.argv[2])
qo = orig["position"][orig["view_indices"][0]:orig["view_indices"][3]+1]
qn, vn, an, t = new["position"], new["velocity"], new["acceleration"], new["time"]
center = new["object_center"]
# limits
vl, al = new["velocity_limits"], new["acceleration_limits"]
fd_v = np.gradient(qn, t, axis=0); fd_a = np.gradient(vn, t, axis=0)
print("peak v/cap %.3f  peak a/cap %.3f  (finite-diff a/cap %.3f)" % ((np.abs(vn)/vl).max(), (np.abs(an)/al).max(), (np.abs(fd_a)/al).max()))
print("v vs finite diff rel err %.4f" % (np.linalg.norm(fd_v - vn)/np.linalg.norm(vn)))
print("closed loop", np.array_equal(qn[0], qn[-1]), " start/end v", np.abs(vn[0]).max(), np.abs(vn[-1]).max())
# joint-space deviation to original dense path
dense = np.vstack([np.linspace(a, b, 20) for a, b in zip(qo[:-1], qo[1:])])
dj = np.array([np.min(np.max(np.abs(dense - x), axis=1)) for x in qn])
print("max joint deviation from Jan's path %.4f rad" % dj.max())
# surface deviation: every mesh vertex vs the nearest original configuration
verts_o = []
sub_o = dense[::4]
cam_o = np.array([robot.fk(x)["camera"][:3, 3] for x in sub_o])
tri_o = [robot.triangles(x)[:, 0, :][::5] for x in sub_o]
worst, worst_aim = 0, 0
for x in qn[::3]:
    cam = robot.fk(x)["camera"]
    k = np.argmin(np.linalg.norm(cam_o - cam[:3, 3], axis=1))
    lo, hi = max(0, k - 3), min(len(sub_o), k + 4)
    tri = robot.triangles(x)[:, 0, :][::5]
    worst = max(worst, min(np.linalg.norm(tri - tri_o[j], axis=1).max() for j in range(lo, hi)))
    fwd = cam[:3, 0]; dirn = center - cam[:3, 3]; dirn /= np.linalg.norm(dirn)
    worst_aim = max(worst_aim, np.degrees(np.arccos(np.clip(fwd @ dirn, -1, 1))))
print("max robot-surface deviation from nearest original pose %.1f mm" % (1000 * worst))
print("max camera aim error %.3f deg (Jan's limit 0.5)" % worst_aim)
