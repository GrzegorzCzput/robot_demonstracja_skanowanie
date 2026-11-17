"""Camera paths for the wide scan and the ascending constant-distance arc."""
from pathlib import Path

import numpy as np
from scipy.interpolate import PchipInterpolator

MODE_LABELS = {"scan": "Four-view scan", "upward_arc": "Upward arc"}
MODE_FILES = {"scan": "scan_4_positions.npz", "upward_arc": "scan_upward_arc.npz"}


def view_parameters(cfg, mode):
    if mode == "scan":
        return np.asarray(cfg["view_angles_deg"], dtype=float)
    if mode == "upward_arc":
        if "view_elevations_deg" not in cfg["upward_arc"]:
            raise ValueError("Resolve the upward arc's reachable endpoint before requesting view parameters")
        return np.asarray(cfg["upward_arc"]["view_elevations_deg"], dtype=float)
    raise ValueError(f"Unknown motion mode: {mode}")


def camera_path(cfg, mode, parameters):
    parameters = np.atleast_1d(parameters).astype(float)
    center = np.asarray(cfg["object_center"], dtype=float)
    if mode == "scan":
        knots = view_parameters(cfg, mode)
        radii = PchipInterpolator(knots, cfg["view_radii"])(parameters)
        heights = PchipInterpolator(knots, cfg["view_heights"])(parameters)
        angles = np.deg2rad(parameters)
        positions = np.column_stack([center[0] + radii * np.cos(angles),
                                     center[1] + radii * np.sin(angles), heights])
        rolls = PchipInterpolator(knots, cfg["view_rolls_deg"])(parameters)
    elif mode == "upward_arc":
        settings = cfg["upward_arc"]
        radius = settings["radius"]
        azimuth = np.deg2rad(settings["azimuth_deg"])
        elevation = np.deg2rad(parameters)
        positions = center + radius * np.column_stack([
            np.cos(elevation) * np.cos(azimuth), np.cos(elevation) * np.sin(azimuth),
            np.sin(elevation)])
        rolls = np.zeros_like(parameters)
    else:
        raise ValueError(f"Unknown motion mode: {mode}")
    return positions, rolls


def trajectory_file(root: Path, mode: str):
    return root / "outputs" / MODE_FILES[mode]
