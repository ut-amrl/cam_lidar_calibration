#!/usr/bin/env python3
"""Assess ACFR captures and write explicit camera-to-LiDAR extrinsics, without a GUI."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def aggregate(csv):
    rows = np.genfromtxt(csv, delimiter=",", skip_header=1, ndmin=2)
    if rows.shape[1] != 6 or len(rows) < 3 or not np.isfinite(rows).all():
        raise ValueError("Need at least three finite optimized triplets with roll,pitch,yaw,x,y,z")
    rows[:, :3] = rows[0, :3] + np.angle(np.exp(1j * (rows[:, :3] - rows[0, :3])))
    for column in range(6):
        deviation = rows[:, column].std()
        if deviation > 1e-12:
            rows = rows[np.abs((rows[:, column] - rows[:, column].mean()) / deviation) < 2]
        if len(rows) < 3:
            raise ValueError("Fewer than three triplets remain after outlier rejection; collect more diverse poses")
    means, spread = rows.mean(axis=0), rows.std(axis=0, ddof=1)
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", means[:3]).as_matrix()
    transform[:3, 3] = means[3:]
    return transform, spread, len(rows)


def assess(csv):
    csv = Path(csv)
    transform, spread, retained = aggregate(csv)
    camera = yaml.safe_load((csv.parent / "camera_info.yaml").read_text())
    camera = camera.get("/**", {}).get("ros__parameters", camera)
    k = np.array(camera["K"], dtype=float).reshape(3, 3)
    d = np.array(camera["D"], dtype=float)
    poses = np.loadtxt(csv.parent / "poses.csv", delimiter=",", ndmin=2)
    if poses.shape[1] != 3 or len(poses) % 19:
        raise ValueError("Expected ACFR poses.csv with 19 rows per capture")
    centers_camera, centers_lidar = poses[0::19] / 1000, poses[6::19] / 1000
    if not len(centers_camera):
        raise ValueError("No captured poses available for assessment")
    inverse = np.linalg.inv(transform)
    predicted_camera = centers_lidar @ inverse[:3, :3].T + inverse[:3, 3]
    if np.any(predicted_camera[:, 2] <= 0) or np.any(centers_camera[:, 2] <= 0):
        raise ValueError("Board center is behind the camera; inspect calibration/frame conventions")

    def project(points):
        if camera["distortion_model"] in ("equidistant", "fisheye"):
            return cv2.fisheye.projectPoints(points.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), k, d)[0].reshape(-1, 2)
        return cv2.projectPoints(points, np.zeros(3), np.zeros(3), k, d)[0].reshape(-1, 2)

    errors_px = np.linalg.norm(project(predicted_camera) - project(centers_camera), axis=1)
    errors_m = np.linalg.norm(predicted_camera - centers_camera, axis=1)
    result = {
        "direction": "camera_optical_to_lidar", "translation_m": transform[:3, 3].tolist(),
        "quaternion_xyzw": Rotation.from_matrix(transform[:3, :3]).as_quat().tolist(),
        "T_lidar_camera": transform.tolist(), "T_camera_lidar": inverse.tolist(),
        "retained_triplets": retained, "triplet_std_rpy_rad_xyz_m": spread.tolist(),
        "captured_poses": len(centers_camera),
        "board_center_error_px": {"mean": float(errors_px.mean()), "max": float(errors_px.max()),
                                  "per_pose": errors_px.tolist()},
        "board_center_error_m": {"mean": float(errors_m.mean()), "max": float(errors_m.max())},
        "source_csv": csv.name,
        "note": "Capture-set center residuals and overlapping-triplet spread are not independent accuracy measurements. Validate on held-out scenes.",
    }
    output = csv.with_suffix(".json")
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(f"Saved {output}")
    print(f"Board-center reprojection error: mean {errors_px.mean():.3f} px, max {errors_px.max():.3f} px")
    print(f"Board-center 3D error: mean {1000 * errors_m.mean():.2f} mm")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()
    assess(args.csv)
