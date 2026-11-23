import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation


def module(name, filename):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).parents[1] / "scripts" / filename)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


workflow = module("workflow", "jazzy_workflow.py")
assessment = module("assessment", "assess_jazzy.py")


class JazzyWorkflowTest(unittest.TestCase):
    def cli(self, session, *arguments):
        with patch.object(sys, "argv", ["workflow", "--session", str(session), *arguments]):
            workflow.main()

    def test_fractional_geometry_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            session = Path(folder)
            self.cli(session, "init", "--board-width-mm", "530.5", "--board-height-mm", "790.25")
            before = (session / "params.yaml").read_text()
            config = json.loads(before)["/**"]["ros__parameters"]["chessboard"]
            self.assertEqual(config["pattern_size"], {"width": 5, "height": 8})
            self.assertEqual(config["square_length"], 83.63)
            with self.assertRaisesRegex(ValueError, "already exists"):
                self.cli(session, "init", "--board-width-mm", "540", "--board-height-mm", "800")
            self.assertEqual(before, (session / "params.yaml").read_text())

    def test_requires_valid_backing_dimensions(self):
        with tempfile.TemporaryDirectory() as folder:
            for width in ("500", "nan"):
                with self.assertRaises(ValueError):
                    self.cli(Path(folder), "init", "--board-width-mm", width, "--board-height-mm", "790")
            self.assertFalse((Path(folder) / "params.yaml").exists())

    def test_yaw_wrap(self):
        with tempfile.TemporaryDirectory() as folder:
            csv = Path(folder) / "result.csv"
            rows = np.zeros((4, 6))
            rows[:, 2] = [np.pi - 0.01, -np.pi + 0.01, np.pi - 0.02, -np.pi + 0.02]
            np.savetxt(csv, rows, delimiter=",", header="roll,pitch,yaw,x,y,z", comments="")
            transform, _, count = assessment.aggregate(csv)
            self.assertEqual(count, 4)
            np.testing.assert_allclose(transform[:3, :3], np.diag([-1, -1, 1]), atol=1e-10)

    def test_transform_direction_and_center_projection(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            csv = folder / "calibration.csv"
            euler = np.array([0.1, -0.2, 0.3])
            rotation = Rotation.from_euler("xyz", euler).as_matrix()
            translation = np.array([0.1, 0.2, 0.3])
            np.savetxt(csv, np.tile(np.r_[euler, translation], (4, 1)), delimiter=",",
                       header="roll,pitch,yaw,x,y,z", comments="")
            poses = np.zeros((3 * 19, 3))
            centers = np.array([[0.1, 0.2, 2], [-0.1, 0.3, 3], [0, 0, 4]])
            poses[0::19] = centers * 1000
            poses[6::19] = (centers @ rotation.T + translation) * 1000
            np.savetxt(folder / "poses.csv", poses, delimiter=",")
            camera = {"K": [100., 0., 50., 0., 100., 50., 0., 0., 1.],
                      "D": [0.1, -0.01, 0., 0., 0.002], "distortion_model": "non-fisheye"}
            (folder / "camera_info.yaml").write_text(json.dumps({"/**": {"ros__parameters": camera}}))
            result = assessment.assess(csv)
            self.assertLess(result["board_center_error_px"]["max"], 1e-10)
            np.testing.assert_allclose(result["translation_m"], translation)
            np.testing.assert_allclose(np.array(result["T_lidar_camera"]) @ result["T_camera_lidar"], np.eye(4), atol=1e-12)
            self.assertTrue(csv.with_suffix(".json").exists())


if __name__ == "__main__":
    unittest.main()
