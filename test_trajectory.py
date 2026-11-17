"""Verify retiming bounds and the saved demonstration without requiring CUDA."""
import json
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from trajectory_math import BLEND, composed_bounds, polynomial_max, retime


class TrajectoryTests(unittest.TestCase):
    def test_blend_stops_through_jerk(self):
        np.testing.assert_allclose(BLEND([0., 1.]), [0., 1.])
        for order in (1, 2, 3):
            np.testing.assert_allclose(BLEND.deriv(order)([0., 1.]), 0., atol=1e-10)
        self.assertAlmostEqual(polynomial_max(BLEND.deriv()), 2.1875)

    def test_curved_path_derivatives_and_bounds(self):
        # Curvature reversal exercises acceleration beyond endpoint distance.
        path = np.array([[0., 0.], [.2, .1], [.4, -.1], [.5, 0.]])
        vl, al = np.array([.12, .2]), np.array([.08, .1])
        q, v, a, spline, duration = retime(path, vl, al, .002)
        np.testing.assert_allclose(q[[0, -1]], path[[0, -1]])
        np.testing.assert_array_equal(v[[0, -1]], 0.)
        np.testing.assert_array_equal(a[[0, -1]], 0.)
        self.assertTrue(np.all(np.abs(v) <= vl))
        self.assertTrue(np.all(np.abs(a) <= al))
        np.testing.assert_allclose(np.gradient(q, .002, axis=0), v, atol=1e-5)
        np.testing.assert_allclose(np.gradient(v, .002, axis=0), a, atol=2e-4)

    def test_saved_scan(self):
        data = np.load(Path(__file__).parent / "outputs/scan_4_positions.npz", allow_pickle=False)
        report = json.loads(str(data["metadata_json"]))
        q, v, a, t = (data[k] for k in ("position", "velocity", "acceleration", "time"))
        self.assertEqual(q.shape, v.shape)
        self.assertEqual(q.shape, a.shape)
        self.assertEqual(q.shape[1], 6)
        self.assertEqual(len(data["view_indices"]), 4)
        self.assertAlmostEqual(t[-1], 18.)
        self.assertEqual(report["camera_frame"], "camera")
        self.assertEqual(report["camera_optical_axis"], "+X")
        self.assertEqual(report["dwell_seconds"], 1.)
        np.testing.assert_array_equal(report["view_angles_deg"], [115, 150, 210, 245])
        np.testing.assert_allclose(report["view_rolls_deg"], [-15, 0, 0, 15], atol=1e-12)
        positions = data["view_positions"]
        azimuths = np.rad2deg(np.arctan2(positions[:, 1] - data["object_center"][1],
                                       positions[:, 0] - data["object_center"][0])) % 360.
        np.testing.assert_allclose(azimuths, [115, 150, 210, 245], atol=1e-6)
        np.testing.assert_allclose(positions[1:3, 2], .34)
        np.testing.assert_allclose(report["view_radii_m"][1:3], .28)
        for i in range(4):
            self.assertEqual(np.sum(data["phase"] == f"view_{i + 1}") - (1 if i == 0 else 0), 50)
        np.testing.assert_array_equal(q[0], q[-1])
        self.assertTrue(np.all(np.diff(t) > 0))
        for key in ("position", "velocity", "acceleration", "camera_position"):
            self.assertTrue(np.isfinite(data[key]).all())
        self.assertTrue(np.all(np.abs(v) <= data["velocity_limits"] + 1e-8))
        self.assertTrue(np.all(np.abs(a) <= data["acceleration_limits"] + 1e-8))
        dwell = np.char.startswith(data["phase"], "view_")
        np.testing.assert_allclose(v[dwell], 0.)
        np.testing.assert_allclose(a[dwell], 0.)
        dt = report["sample_dt"]
        velocity_fd_error = np.linalg.norm(np.gradient(q, dt, axis=0) - v)
        self.assertLess(velocity_fd_error / np.linalg.norm(v), .0025)
        # Integrate the 50 Hz acceleration samples: differentiating velocity at
        # individual changing-jerk knots unnecessarily amplifies sampling error.
        integrated_v = np.vstack([v[0], v[0] + np.cumsum(.5 * (a[:-1] + a[1:]) * dt, axis=0)])
        self.assertLess(np.linalg.norm(integrated_v - v) / np.linalg.norm(v), .015)
        self.assertGreater(report["min_world_clearance_m"], 0.)
        self.assertGreater(report["min_self_clearance_m"], 0.)
        indices = data["view_indices"]
        forward = Rotation.from_quat(data["camera_quaternion_wxyz"][indices], scalar_first=True).apply(np.tile([1., 0., 0.], (4, 1)))
        target = data["object_center"] - data["camera_position"][indices]
        target /= np.linalg.norm(target, axis=1)[:, None]
        self.assertTrue(np.all(np.sum(forward * target, axis=1) > np.cos(np.deg2rad(2))))

    def test_continuous_composition_bounds_and_fixed_duration(self):
        path = np.array([[0., 0.], [.2, .1], [.4, -.1], [.5, 0.]])
        vl, al = np.ones(2), np.ones(2)
        _, _, _, spline, minimum = retime(path, vl, al, .01)
        vb, ab = composed_bounds(spline)
        u = np.linspace(0., 1., 100001)
        h = np.clip(BLEND(u), 0., 1.)
        hd, hdd = BLEND.deriv()(u)[:, None], BLEND.deriv(2)(u)[:, None]
        self.assertTrue(np.all(np.abs(spline(h, 1) * hd) <= vb + 1e-10))
        self.assertTrue(np.all(np.abs(spline(h, 2) * hd**2 + spline(h, 1) * hdd) <= ab + 1e-10))
        with self.assertRaises(ValueError):
            retime(path, vl, al, .01, requested_duration=.01)
        *_, duration = retime(path, vl, al, .01, requested_duration=minimum + 1.)
        self.assertAlmostEqual(duration, minimum + 1.)

    def test_arc_length_retiming(self):
        path = np.array([[0., 0.], [.01, .001], [.15, .08], [.4, .1], [.5, .12]])
        vl, al = np.array([.3, .3]), np.array([.4, .4])
        dt = .0002
        q, v, a, _, _ = retime(path, vl, al, dt, parameterization="arc_length")
        np.testing.assert_allclose(q[[0, -1]], path[[0, -1]])
        self.assertTrue(np.all(np.abs(v) <= vl))
        self.assertTrue(np.all(np.abs(a) <= al))
        np.testing.assert_allclose(np.gradient(q, dt, axis=0), v, atol=1e-6)
        np.testing.assert_allclose(np.gradient(v, dt, axis=0), a, atol=1e-4)

    def test_saved_camera_down_axis(self):
        data = np.load(Path(__file__).parent / "outputs/scan_4_positions.npz", allow_pickle=False)
        rotations = Rotation.from_quat(data["camera_quaternion_wxyz"], scalar_first=True).as_matrix()
        forward, image_down = rotations[:, :, 0], -rotations[:, :, 2]
        np.testing.assert_allclose(data["image_down_world"], image_down, atol=1e-7)
        self.assertTrue(np.all(image_down[:, 2] < -.7))
        world_down = np.broadcast_to([0., 0., -1.], image_down.shape)
        ideal_down = world_down - forward * np.sum(world_down * forward, axis=1)[:, None]
        ideal_down /= np.linalg.norm(ideal_down, axis=1)[:, None]
        absolute_roll = np.rad2deg(np.arccos(np.clip(np.sum(ideal_down * image_down, axis=1), -1., 1.)))
        self.assertLessEqual(absolute_roll.max(), 20.)
        direction = data["object_center"] - data["camera_position"]
        direction /= np.linalg.norm(direction, axis=1)[:, None]
        aim_error = np.rad2deg(np.arccos(np.clip(np.sum(forward * direction, axis=1), -1., 1.)))
        self.assertLess(aim_error.max(), .5)

    def test_saved_upward_arc(self):
        data = np.load(Path(__file__).parent / "outputs/scan_upward_arc.npz", allow_pickle=False)
        report = json.loads(str(data["metadata_json"]))
        self.assertEqual(report["mode"], "upward_arc")
        self.assertFalse(report["loop_allowed"])
        self.assertFalse(report["closed_loop"])
        self.assertAlmostEqual(data["time"][-1], 18.)
        self.assertFalse(np.array_equal(data["position"][0], data["position"][-1]))
        distance = np.linalg.norm(data["camera_position"] - data["object_center"], axis=1)
        np.testing.assert_allclose(distance, .38, atol=.001)
        self.assertGreaterEqual(np.min(np.diff(data["camera_position"][:, 2])), -2e-5)
        self.assertGreater(data["camera_position"][-1, 2] - data["camera_position"][0, 2], .20)
        self.assertGreater(data["camera_position"][-1, 2], .46)
        self.assertGreater(report["view_parameters_deg"][-1], 74.)
        self.assertAlmostEqual(report["reachable_elevation_limit_deg"] - report["view_parameters_deg"][-1],
                               report["endpoint_margin_deg"])
        self.assertLessEqual(report["first_unreachable_elevation_deg"] - report["reachable_elevation_limit_deg"], .011)
        np.testing.assert_allclose(data["camera_position"][:, 1], data["object_center"][1], atol=.001)
        self.assertFalse(np.any(data["phase"] == "move_4_to_1"))
        for i in range(4):
            self.assertEqual(np.sum(data["phase"] == f"view_{i + 1}") - (1 if i == 3 else 0), 50)
        self.assertTrue(np.all(np.abs(data["velocity"]) <= data["velocity_limits"] + 1e-8))
        self.assertTrue(np.all(np.abs(data["acceleration"]) <= data["acceleration_limits"] + 1e-8))
        dwell = np.char.startswith(data["phase"], "view_")
        np.testing.assert_array_equal(data["velocity"][dwell], 0.)
        np.testing.assert_array_equal(data["acceleration"][dwell], 0.)
        self.assertLess(np.abs(data["camera_roll_deg"]).max(), .5)
        self.assertLess(report["max_continuous_aim_error_deg"], .5)
        self.assertGreater(report["min_world_clearance_m"], 0.)
        self.assertGreater(report["min_self_clearance_m"], 0.)


if __name__ == "__main__":
    unittest.main()
