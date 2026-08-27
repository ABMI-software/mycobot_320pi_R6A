import sys
import unittest
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import adaptive_pick_by_demo as adaptive  # noqa: E402


def demo(x, y, value):
    return {
        "ball": {"base_xy_m": [x, y], "pixel_uv": [100 + x, 200 + y]},
        "pick_approach_deg": [value] * 6,
        "pick_deg": [value + 1] * 6,
    }


def dataset(demos):
    return {
        "schema_version": 1,
        "input_space": "base_xy_m",
        "calibration": {},
        "safety": {"min_demo_separation_m": 0.01,
                   "max_nearest_demo_distance_m": 2.0},
        "demonstrations": demos,
    }


class GeometryTests(unittest.TestCase):
    def test_manual_ros_image_decode_honors_step_and_rgb_order(self):
        class Image:
            height = 1
            width = 2
            step = 8  # two RGB pixels plus two padding bytes
            encoding = "rgb8"
            data = bytes([10, 20, 30, 40, 50, 60, 99, 99])

        frame = adaptive.decode_ros_image_bgr8(Image())
        self.assertEqual(frame.shape, (1, 2, 3))
        self.assertEqual(frame.tolist(), [[[30, 20, 10], [60, 50, 40]]])

    def test_manual_ros_image_decode_rejects_short_buffer(self):
        class Image:
            height = 2
            width = 2
            step = 6
            encoding = "bgr8"
            data = bytes(6)

        with self.assertRaisesRegex(adaptive.DemoError, "tronqué"):
            adaptive.decode_ros_image_bgr8(Image())

    def test_hull_accepts_inside_and_boundary_but_not_outside(self):
        hull = adaptive.convex_hull([(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 0.5)])
        self.assertTrue(adaptive.point_in_convex_hull((0.5, 0.5), hull))
        self.assertTrue(adaptive.point_in_convex_hull((1.0, 0.4), hull))
        self.assertFalse(adaptive.point_in_convex_hull((1.01, 0.4), hull))

    def test_workspace_clearance_has_sign_and_margin_distance(self):
        square = [(0, 0), (1, 0), (1, 1), (0, 1)]
        self.assertAlmostEqual(adaptive.workspace_clearance((0.5, 0.5), square), 0.5)
        self.assertLess(adaptive.workspace_clearance((1.1, 0.5), square), 0)

    def test_ros_response_prefixes_ignore_joint_sync_noise(self):
        self.assertEqual(adaptive.Ros2Bridge._expected_prefix("send_angles"), "OK: angles")
        self.assertEqual(adaptive.Ros2Bridge._expected_prefix("get_pro_gripper_status"),
                         "PRO_GRIPPER_STATUS:")

    def test_gripper_ack_must_match_requested_angle(self):
        command = {"action": "pro_gripper_angle", "angle": 20}
        adaptive.validate_command_response(command, "OK: pro gripper angle 20")
        with self.assertRaises(adaptive.ExecutionError):
            adaptive.validate_command_response(command, "OK: pro gripper angle 100")

    def test_real_calibration_projects_to_ball_centre_plane(self):
        calibration = adaptive.REPO / "training/calibration"
        data = adaptive.new_dataset(
            calibration / "cam_0.meta.json",
            calibration / "arducam_extrinsic_handeye.yaml",
            calibration / "workspace_markers.yaml",
            adaptive.DEFAULT_PROJECTION_Z_M,
            0.025,
        )
        transform = adaptive.load_extrinsic_transform(
            calibration / "arducam_extrinsic_handeye.yaml")
        xyz = adaptive.project_raw_pixel_to_plane(
            calibration / "cam_0.meta.json", (160, 160), transform,
            adaptive.DEFAULT_PROJECTION_Z_M)
        self.assertAlmostEqual(xyz[2], 0.0335, places=6)
        self.assertGreater(xyz[0], 0.14)

    def test_live_boundary_overrides_old_marker_positions(self):
        calibration = adaptive.REPO / "training/calibration"
        data = adaptive.new_dataset(
            calibration / "cam_0.meta.json",
            calibration / "arducam_extrinsic_handeye.yaml",
            calibration / "workspace_markers.yaml",
            adaptive.DEFAULT_PROJECTION_Z_M,
            0.025,
        )
        transform = adaptive.load_extrinsic_transform(
            calibration / "arducam_extrinsic_handeye.yaml")
        xyz = adaptive.project_raw_pixel_to_plane(
            calibration / "cam_0.meta.json", (160, 160), transform,
            adaptive.DEFAULT_PROJECTION_Z_M)
        x, y = xyz[:2]
        live_polygon = [(x - 0.10, y - 0.10), (x + 0.10, y - 0.10),
                        (x + 0.10, y + 0.10), (x - 0.10, y + 0.10)]
        projected = adaptive.project_pixel_with_extrinsic(
            data, (160, 160), transform, live_polygon)
        self.assertAlmostEqual(projected[0], x)
        with self.assertRaisesRegex(adaptive.DemoError, "hors zone ArUco"):
            adaptive.project_pixel_with_extrinsic(
                data, (160, 160), transform,
                [(x + 1, y + 1), (x + 1.2, y + 1),
                 (x + 1.2, y + 1.2), (x + 1, y + 1.2)])

    def test_live_pnp_requires_four_markers_and_has_low_rms(self):
        import cv2
        import numpy as np
        import yaml

        calibration = adaptive.REPO / "training/calibration"
        K, dist = adaptive.load_intrinsic_parameters(calibration / "cam_0.meta.json")
        workspace = yaml.safe_load((calibration / "workspace_markers.yaml").read_text())
        world = {int(key): value for key, value in workspace["markers"].items()}
        extrinsic = yaml.safe_load(
            (calibration / "arducam_extrinsic_markers.yaml").read_text())
        T_cam_world = np.linalg.inv(np.asarray(extrinsic["T_world_cam"], dtype=np.float64))
        rvec, _ = cv2.Rodrigues(T_cam_world[:3, :3])
        image, _ = cv2.projectPoints(
            np.asarray(list(world.values()), dtype=np.float64), rvec,
            T_cam_world[:3, 3], K, dist)
        centers = {marker_id: point for marker_id, point in zip(world, image.reshape(-1, 2))}
        _, rms, ids = adaptive.solve_live_marker_extrinsic(centers, world, K, dist)
        self.assertEqual(len(ids), 4)
        self.assertLess(rms, 1e-3)
        centers.pop(next(iter(centers)))
        with self.assertRaisesRegex(adaptive.DemoError, "4 ArUco"):
            adaptive.solve_live_marker_extrinsic(centers, world, K, dist)


class InterpolationTests(unittest.TestCase):
    def setUp(self):
        self.data = dataset([demo(0, 0, 0), demo(1, 0, 10), demo(0, 1, 20)])

    def test_exact_demonstration_is_reproduced(self):
        prediction = adaptive.interpolate(self.data, (1, 0))
        self.assertEqual(prediction.pick_approach_deg, (10.0,) * 6)
        self.assertEqual(prediction.pick_deg, (11.0,) * 6)
        self.assertEqual(prediction.weights, (0.0, 1.0, 0.0))

    def test_idw_interpolates_every_joint(self):
        prediction = adaptive.interpolate(self.data, (1 / 3, 1 / 3))
        self.assertAlmostEqual(sum(prediction.weights), 1.0)
        for joint in prediction.pick_deg:
            self.assertGreater(joint, 1.0)
            self.assertLess(joint, 21.0)

    def test_extrapolation_is_refused(self):
        with self.assertRaisesRegex(adaptive.DemoError, "extrapolation"):
            adaptive.interpolate(self.data, (0.8, 0.8))

    def test_three_collinear_demonstrations_are_not_ready(self):
        bad = dataset([demo(0, 0, 0), demo(1, 0, 10), demo(2, 0, 20)])
        with self.assertRaisesRegex(adaptive.DemoError, "non collineaires"):
            adaptive.interpolate(bad, (1, 0))

    def test_raw_pixel_input_space_is_rejected(self):
        bad = self.data | {"input_space": "pixel_uv"}
        with self.assertRaisesRegex(adaptive.DemoError, "pixels bruts"):
            adaptive.validate_dataset(bad)

    def test_generated_plan_contains_joint_commands_only(self):
        prediction = adaptive.interpolate(self.data, (0, 0))
        plan = adaptive.prediction_plan(prediction, (100, 200), 20, 100, 20)
        robot_steps = [step for step in plan["steps"] if step["action"].startswith("send_")]
        self.assertTrue(robot_steps)
        self.assertTrue(all(step["action"] == "send_angles" for step in robot_steps))
        self.assertFalse(any("coords" in step for step in plan["steps"]))

    def test_real_execution_rejects_static_demo_localization(self):
        with self.assertRaisesRegex(adaptive.DemoError, "execution interdite"):
            adaptive.validate_live_execution_dataset(self.data)

    def test_real_execution_accepts_eye_to_hand_with_live_boundaries(self):
        ready = dataset([demo(0, 0, 0), demo(1, 0, 10), demo(0, 1, 20)])
        for item in ready["demonstrations"]:
            item["ball"]["localization"] = {
                "source": "eye_to_hand+aruco_boundary_live",
                "marker_ids": [19, 23, 25, 26],
                "max_marker_spread_px": 0.5,
                "workspace_base_xy": [[0, 0], [1, 0], [1, 1], [0, 1]],
            }
        adaptive.validate_live_execution_dataset(ready)

    def test_query_too_far_from_nearest_demo_is_rejected(self):
        guarded = dataset([demo(0, 0, 0), demo(1, 0, 10), demo(0, 1, 20)])
        guarded["safety"]["max_nearest_demo_distance_m"] = 0.1
        with self.assertRaisesRegex(adaptive.DemoError, "trop loin"):
            adaptive.interpolate(guarded, (1 / 3, 1 / 3))

    def test_demo_minimum_separation_is_enforced(self):
        close = dataset([demo(0, 0, 0), demo(0.005, 0, 10), demo(0, 1, 20)])
        with self.assertRaisesRegex(adaptive.DemoError, "trop proche"):
            adaptive.validate_dataset(close)


if __name__ == "__main__":
    unittest.main()
