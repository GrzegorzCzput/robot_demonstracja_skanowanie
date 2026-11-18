"""Render the upward-arc trajectory as a side-view figure with the real robot meshes.

    python render_upward_arc.py            # writes outputs/upward_arc_motion.png

Needs only numpy, scipy and matplotlib (no GPU, cuRobo or viewer).
"""
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Rectangle
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent
URDF = ROOT / "third_party/trlc-dk1/urdf/follower/TRLC-DK1-Follower.urdf"
TRAJECTORY = ROOT / "outputs/scan_upward_arc.npz"
OUTPUT = ROOT / "outputs/upward_arc_motion.png"

INK, MUTED, GRID = "#1f2328", "#6e7781", "#d0d7de"
ROBOT, ARC, RAY, OBJECT, TABLE = "#57606a", "#0969da", "#bf8700", "#2da44e", "#c9b28f"


def load_stl(path):
    data = path.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:512]:
        v = [list(map(float, line.split()[1:])) for line in data.decode().splitlines()
             if line.strip().startswith("vertex")]
        return np.asarray(v).reshape(-1, 3, 3)
    count = int(np.frombuffer(data, np.uint32, 1, 80)[0])
    records = np.frombuffer(data, np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count, 84)
    return records["v"].astype(float)


def matrix(xyz="0 0 0", rpy="0 0 0"):
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler("xyz", list(map(float, rpy.split()))).as_matrix()
    m[:3, 3] = list(map(float, xyz.split()))
    return m


class Robot:
    def __init__(self, urdf):
        root = ET.parse(urdf).getroot()
        self.joints = []
        for j in root.findall("joint"):
            o, axis = j.find("origin"), j.find("axis")
            self.joints.append(dict(
                name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                child=j.find("child").get("link"),
                origin=matrix(o.get("xyz", "0 0 0"), o.get("rpy", "0 0 0")),
                axis=np.array(list(map(float, axis.get("xyz").split()))) if axis is not None else None))
        self.meshes = {}
        for link in root.findall("link"):
            mesh = link.find("collision/geometry/mesh")
            if mesh is not None:
                o = link.find("collision/origin")
                local = matrix(o.get("xyz", "0 0 0"), o.get("rpy", "0 0 0")) if o is not None else np.eye(4)
                tris = load_stl(urdf.parent / mesh.get("filename"))
                self.meshes[link.get("name")] = tris @ local[:3, :3].T + local[:3, 3]

    def fk(self, q):
        """Link poses for arm joints q (joint1..joint6); gripper closed at zero."""
        values = {f"joint{i + 1}": v for i, v in enumerate(q)}
        poses = {"base_link": np.eye(4)}
        for j in self.joints:  # URDF lists joints parent-first
            m = poses[j["parent"]] @ j["origin"]
            if j["type"] == "revolute":
                r = np.eye(4)
                r[:3, :3] = Rotation.from_rotvec(j["axis"] * values[j["name"]]).as_matrix()
                m = m @ r
            poses[j["child"]] = m
        return poses

    def triangles(self, q):
        poses = self.fk(q)
        return np.concatenate([tris @ poses[name][:3, :3].T + poses[name][:3, 3]
                               for name, tris in self.meshes.items()])


def draw_robot(ax, tris, color=ROBOT, alpha=1.0, zorder=3):
    """Orthographic view from -Y: x right, z up; painter-sorted, normal-shaded."""
    tris = tris[np.argsort(-tris[:, :, 1].mean(1))]  # far (large y) first
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    light = np.array([-0.35, -0.75, 0.55])
    shade = 0.45 + 0.55 * np.abs(n @ (light / np.linalg.norm(light)))
    base = np.array(matplotlib.colors.to_rgb(color))
    colors = np.clip(base[None] * shade[:, None] + (1 - shade[:, None]) * 0.12, 0, 1)
    pc = PolyCollection(tris[:, :, [0, 2]], facecolors=np.c_[colors, np.full(len(colors), alpha)],
                        edgecolors="none", zorder=zorder, antialiased=False)
    ax.add_collection(pc)


def main():
    robot = Robot(URDF)
    with np.load(TRAJECTORY, allow_pickle=False) as d:
        data = {k: d[k] for k in d.files}
    report = json.loads(str(data["metadata_json"]))
    t, q, cam = data["time"], data["position"], data["camera_position"]
    center, views = data["object_center"], data["view_indices"]

    # Sanity check: our FK must reproduce the planner's saved camera positions.
    fk_cam = np.array([robot.fk(q[i])["camera"][:3, 3] for i in range(0, len(q), 50)])
    error = np.abs(fk_cam - cam[::50]).max()
    if error > 1e-4:
        raise RuntimeError(f"FK mismatch with saved trajectory: {error * 1000:.2f} mm")

    radius = report["arc_radius_m"]
    elevation = np.rad2deg(np.arcsin(np.clip((cam[:, 2] - center[2]) / radius, -1, 1)))
    size = 0.2
    # Panel width from the arm's real extent over all views (it folds back behind the base).
    poses = [robot.triangles(q[i]) for i in views]
    xlim = (min(p[:, :, 0].min() for p in poses) - 0.02, center[0] + size / 2 + 0.03)
    full = np.deg2rad(np.linspace(0, 90, 200))
    arc_full = center[[0, 2]] + radius * np.c_[-np.cos(full), np.sin(full)]

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.edgecolor": GRID,
                         "axes.labelcolor": MUTED, "xtick.color": MUTED, "ytick.color": MUTED})
    fig = plt.figure(figsize=(16, 8.4), facecolor="white")
    gs = fig.add_gridspec(2, 4, height_ratios=[1.75, 1], hspace=0.16, wspace=0.04,
                          left=0.045, right=0.955, top=0.87, bottom=0.075)
    fig.text(0.045, 0.955, "TRLC-DK1 · Upward arc", fontsize=20, weight="bold", color=INK)
    fig.text(0.045, 0.912,
             f"Camera stays {100 * radius:.0f} cm from the object center and rises from "
             f"{report['view_parameters_deg'][0]:.0f}° to {report['view_parameters_deg'][-1]:.1f}° elevation "
             f"(reachable limit {report['reachable_elevation_limit_deg']:.1f}°), level picture, "
             f"four 1 s captures, stops at the top. Side view from −Y.",
             fontsize=11, color=MUTED)

    for k, idx in enumerate(views):
        ax = fig.add_subplot(gs[0, k])
        ax.set_aspect("equal")
        ax.set_xlim(*xlim)
        ax.set_ylim(-0.06, 0.72)
        ax.axis("off")
        ax.add_patch(Rectangle((xlim[0], -0.06), xlim[1] - xlim[0], 0.03, color=TABLE, zorder=1, lw=0))
        ax.add_patch(Rectangle((center[0] - size / 2, center[2] - size / 2), size, size,
                               facecolor=OBJECT, alpha=0.25, edgecolor=OBJECT, lw=1.2, zorder=2))
        ax.plot(*arc_full.T, color=ARC, lw=1, ls=(0, (3, 3)), alpha=0.5, zorder=2)
        ax.plot(cam[:, 0], cam[:, 2], color=ARC, lw=1.6, alpha=0.35, zorder=2)
        ax.plot(cam[:idx + 1, 0], cam[:idx + 1, 2], color=ARC, lw=3, zorder=5)
        for j, other in enumerate(views):
            ax.plot(*cam[other, [0, 2]], "o", ms=6 if j != k else 9, zorder=6,
                    mfc=ARC if j <= k else "white", mec=ARC, mew=1.5)
        draw_robot(ax, poses[k])
        ax.plot([cam[idx, 0], center[0]], [cam[idx, 2], center[2]], color=RAY, lw=1.6, zorder=4)
        ax.plot(*center[[0, 2]], "+", color=INK, ms=10, mew=1.5, zorder=6)
        ax.text(0.02, 0.98, f"View {k + 1}", transform=ax.transAxes, fontsize=14, weight="bold",
                color=INK, va="top")
        ax.text(0.02, 0.90, f"{elevation[idx]:.1f}° elevation · camera Z {100 * cam[idx, 2]:.1f} cm\n"
                            f"t = {t[idx]:.1f}–{t[idx] + report['dwell_seconds']:.1f} s",
                transform=ax.transAxes, fontsize=10, color=MUTED, va="top", linespacing=1.4)

    ax = fig.add_subplot(gs[1, :])
    ax.plot(t, elevation, color=ARC, lw=2.2)
    ax.set_ylabel("Camera elevation (°)")
    ax.set_ylim(15, 80)
    ax.set_xlabel("Time (s)")
    ax.set_xlim(0, t[-1])
    ax.grid(color=GRID, lw=0.6)
    ax.spines[["top", "right"]].set_visible(False)
    for k, idx in enumerate(views):
        ax.axvspan(t[idx], t[idx] + report["dwell_seconds"], color=ARC, alpha=0.1, lw=0)
        ax.text(t[idx] + report["dwell_seconds"] / 2, elevation[idx] - 5, f"View {k + 1}",
                ha="center", va="top", fontsize=9, color=INK)
    ax2 = ax.twinx()
    ax2.plot(t, 100 * cam[:, 2], color=RAY, lw=1.6, ls="--")
    ax2.set_ylabel("Camera height Z (cm)", color=RAY)
    ax2.spines[["top"]].set_visible(False)
    lo, hi = ax.get_ylim()
    ax2.set_ylim(100 * (center[2] + radius * np.sin(np.deg2rad(lo))),
                 100 * (center[2] + radius * np.sin(np.deg2rad(hi))))
    ax.plot([], [], color=RAY, lw=1.6, ls="--", label="Camera height")
    ax.plot([], [], color=ARC, lw=2.2, label="Elevation")
    ax.legend(loc="upper left", frameon=False)

    fig.savefig(OUTPUT, dpi=140)
    print(f"Saved {OUTPUT} (FK check {error * 1000:.3f} mm)")


if __name__ == "__main__":
    main()
