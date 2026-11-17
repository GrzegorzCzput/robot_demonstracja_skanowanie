"""Export joint motion and limit plots from either saved mode."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from scan_geometry import MODE_LABELS, trajectory_file

ROOT = Path(__file__).resolve().parent


def plot_mode(mode):
    with np.load(trajectory_file(ROOT, mode), allow_pickle=False) as data:
        report = json.loads(str(data["metadata_json"]))
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True, constrained_layout=True)
        for ax, field, label in zip(axes, ["position", "velocity", "acceleration"],
                                     ["Position (rad)", "Velocity (rad/s)", "Acceleration (rad/s²)"]):
            for j, name in enumerate(data["joint_names"]):
                ax.plot(data["time"], data[field][:, j], label=str(name), linewidth=1.2)
            if field in ("velocity", "acceleration"):
                limit = data[f"{field}_limits"][0]
                ax.axhline(limit, color="black", linestyle="--", label="Configured cap")
                ax.axhline(-limit, color="black", linestyle="--")
            for i, idx in enumerate(data["view_indices"]):
                t = data["time"][idx]
                ax.axvspan(t, t + report["dwell_seconds"], color="gray", alpha=.12)
                if field == "position":
                    ax.text(t + .1, ax.get_ylim()[1], f"View {i + 1}", va="top", fontsize=8)
            ax.set_ylabel(label)
            ax.grid(alpha=.2)
        axes[0].legend(ncol=6, loc="lower center", bbox_to_anchor=(.5, 1.02))
        axes[-1].set_xlabel("Time (s)")
        fig.suptitle(f"TRLC-DK1 · {MODE_LABELS[mode]} · {report['duration_seconds']:.0f} s · 50 Hz\nFour {report['dwell_seconds']:.0f} s holds · maximum camera roll {report['max_abs_camera_roll_deg']:.1f}°", fontsize=12)
        name = "joint_trajectory.png" if mode == "scan" else "upward_arc_joint_trajectory.png"
        fig.savefig(ROOT / "outputs" / name, dpi=160)
        plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=(*MODE_LABELS, "all"), default="all")
    args = parser.parse_args()
    for mode in MODE_LABELS if args.mode == "all" else (args.mode,):
        plot_mode(mode)
