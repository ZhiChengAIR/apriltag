#!/usr/bin/env python3
"""
AprilTag pose estimation for RealSense D435 (clean, downstream-friendly).

Coordinate Frames
-----------------
OpenCV camera frame (default from solvePnP):
  X: right in image
  Y: down in image
  Z: forward (out of camera)

Robot-friendly camera frame (requested):
  X: right
  Y: forward/out (depth)
  Z: up

We can output in either frame via `frame="opencv"` or `frame="robot"`.
Roll/Pitch/Yaw are ZYX (yaw around Z, pitch around Y, roll around X).
"""

import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
from apriltag import apriltag


def _rotation_to_euler_zyx(R):
    """Convert rotation matrix to Euler angles (roll, pitch, yaw), ZYX order."""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def _transform_from_rt(R, t):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t.flatten()
    return T


def _rt_from_transform(T):
    R = T[:3, :3].copy()
    t = T[:3, 3].reshape(3, 1).copy()
    return R, t


def _invert_rt(R, t):
    R_inv = R.T
    t_inv = -R_inv @ t
    return R_inv, t_inv


def _compose_rt(R1, t1, R2, t2):
    R = R1 @ R2
    t = R1 @ t2 + t1
    return R, t


def _quat_from_R(R):
    """Convert rotation matrix to quaternion (w, x, y, z)."""
    q = np.empty(4, dtype=np.float64)
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q[0] = 0.25 / s
        q[1] = (R[2, 1] - R[1, 2]) * s
        q[2] = (R[0, 2] - R[2, 0]) * s
        q[3] = (R[1, 0] - R[0, 1]) * s
    else:
        i = np.argmax(np.diag(R))
        if i == 0:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[0] = (R[2, 1] - R[1, 2]) / s
            q[1] = 0.25 * s
            q[2] = (R[0, 1] + R[1, 0]) / s
            q[3] = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[0] = (R[0, 2] - R[2, 0]) / s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = 0.25 * s
            q[3] = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[0] = (R[1, 0] - R[0, 1]) / s
            q[1] = (R[0, 2] + R[2, 0]) / s
            q[2] = (R[1, 2] + R[2, 1]) / s
            q[3] = 0.25 * s
    q = q / np.linalg.norm(q)
    return q


def _R_from_quat(q):
    """Convert quaternion (w, x, y, z) to rotation matrix."""
    w, x, y, z = q
    R = np.array([
        [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)]
    ], dtype=np.float64)
    return R


def _average_quaternions(quats):
    """Average quaternions via Markley method."""
    A = np.zeros((4, 4), dtype=np.float64)
    for q in quats:
        if q[0] < 0:
            q = -q
        A += np.outer(q, q)
    eigvals, eigvecs = np.linalg.eigh(A)
    q_avg = eigvecs[:, np.argmax(eigvals)]
    if q_avg[0] < 0:
        q_avg = -q_avg
    return q_avg / np.linalg.norm(q_avg)


class AprilTagPoseTracker:
    def __init__(
        self,
        tag_size_mm=31.0,
        tag_family="tagStandard41h12",
        tag_id=0,
        config_file="realsense_D435.yaml",
        resolution=None,
    ):
        self.tag_size_mm = float(tag_size_mm)
        self.tag_size_m = self.tag_size_mm / 1000.0
        self.tag_family = tag_family
        self.tag_id = int(tag_id)

        self.camera_params = self._load_camera_config(config_file)

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        stream_config = self.camera_params.get("stream", {})
        width = stream_config.get("width", 640)
        height = stream_config.get("height", 480)
        fps = stream_config.get("fps", 30)

        resolution_map = {
            "low": (640, 480),
            "medium": (1280, 720),
            "high": (1920, 1080),
        }
        if resolution:
            if resolution not in resolution_map:
                raise ValueError(f"Unknown resolution preset: {resolution}")
            width, height = resolution_map[resolution]

        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.profile = self.pipeline.start(self.config)

        color_stream = self.profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        self.fx = intrinsics.fx
        self.fy = intrinsics.fy
        self.cx = intrinsics.ppx
        self.cy = intrinsics.ppy
        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float64)
        self.dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float64)

        self._configure_camera_parameters()

        self.detector = apriltag(self.tag_family)

        half = self.tag_size_m / 2.0
        self.object_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float64)

        # OpenCV -> Robot frame transform
        self.T_cam_to_robot = np.array([
            [1,  0,  0],
            [0,  0,  1],
            [0, -1,  0]
        ], dtype=np.float64)

    def close(self):
        self.pipeline.stop()

    def _load_camera_config(self, config_file):
        config_path = Path(config_file)
        if not config_path.exists():
            return {
                "stream": {"width": 640, "height": 480, "fps": 30},
                "rgb_sensor": {"enable_auto_exposure": 1},
            }
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        if "realsense" not in config:
            return {"stream": {"width": 640, "height": 480, "fps": 30}}
        return config["realsense"]

    def _find_rgb_sensor(self):
        device = self.profile.get_device()
        for sensor in device.query_sensors():
            if sensor.supports(rs.camera_info.name):
                sensor_name = sensor.get_info(rs.camera_info.name)
                if "RGB Camera" in sensor_name:
                    return sensor
        return None

    def _configure_camera_parameters(self):
        rgb_sensor = self._find_rgb_sensor()
        if rgb_sensor is None:
            return
        sensor_params = self.camera_params.get("rgb_sensor", {})
        if not sensor_params:
            return
        param_map = {
            "enable_auto_exposure": (rs.option.enable_auto_exposure),
            "enable_auto_white_balance": (rs.option.enable_auto_white_balance),
            "exposure": (rs.option.exposure),
            "gain": (rs.option.gain),
            "white_balance": (rs.option.white_balance),
            "brightness": (rs.option.brightness),
            "contrast": (rs.option.contrast),
            "gamma": (rs.option.gamma),
            "saturation": (rs.option.saturation),
            "sharpness": (rs.option.sharpness),
        }
        for yaml_key, rs_option in param_map.items():
            if yaml_key in sensor_params and rgb_sensor.supports(rs_option):
                try:
                    rgb_sensor.set_option(rs_option, sensor_params[yaml_key])
                except Exception:
                    pass

    def _get_corners_from_detection(self, detection):
        raw_corners = detection["lb-rb-rt-lt"]
        corners = np.array([
            raw_corners[3],  # lt
            raw_corners[2],  # rt
            raw_corners[1],  # rb
            raw_corners[0],  # lb
        ], dtype=np.float64)
        return corners

    def _refine_corners_subpix(self, gray, corners):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
        corners_np = corners.astype(np.float32).reshape(-1, 1, 2)
        cv2.cornerSubPix(gray, corners_np, (5, 5), (-1, -1), criteria)
        return corners_np.reshape(-1, 2)

    def _estimate_pose(self, corners):
        success, rvec, tvec = cv2.solvePnP(
            self.object_points,
            corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        return success, rvec, tvec

    def _refine_pose(self, corners, rvec, tvec):
        try:
            rvec_refined, tvec_refined = cv2.solvePnPRefineLM(
                self.object_points,
                corners,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
            )
            return rvec_refined, tvec_refined
        except Exception:
            return rvec, tvec

    def _compute_reproj_error(self, corners, rvec, tvec):
        projected, _ = cv2.projectPoints(
            self.object_points, rvec, tvec,
            self.camera_matrix, self.dist_coeffs
        )
        projected = projected.reshape(-1, 2)
        diff = corners - projected
        error = np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))
        return error

    def _compute_tag_size_px(self, corners):
        p0, p1, p2, p3 = corners
        width_top = np.linalg.norm(p1 - p0)
        width_bottom = np.linalg.norm(p2 - p3)
        height_left = np.linalg.norm(p3 - p0)
        height_right = np.linalg.norm(p2 - p1)
        return (width_top + width_bottom + height_left + height_right) / 4.0

    def _convert_frame(self, R, t, frame):
        if frame == "opencv":
            return R, t
        if frame == "robot":
            T = self.T_cam_to_robot
            R_conv = T @ R @ T.T
            t_conv = T @ t
            return R_conv, t_conv
        raise ValueError(f"Unknown frame: {frame}")

    def get_tag_pose(self, frame="robot"):
        """
        Detects a tag and returns its pose (tag in camera frame).
        Returns None if not detected.
        """
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)
        target = None
        for det in detections:
            if det["id"] == self.tag_id:
                target = det
                break
        if target is None:
            return None

        corners = self._get_corners_from_detection(target)
        corners = self._refine_corners_subpix(gray, corners)

        success, rvec, tvec = self._estimate_pose(corners)
        if not success:
            return None

        rvec, tvec = self._refine_pose(corners, rvec, tvec)
        reproj_err = self._compute_reproj_error(corners, rvec, tvec)
        tag_size_px = self._compute_tag_size_px(corners)

        R, _ = cv2.Rodrigues(rvec)
        R, t = self._convert_frame(R, tvec, frame)

        roll, pitch, yaw = _rotation_to_euler_zyx(R)
        position_mm = (t.flatten() * 1000.0).tolist()

        return {
            "R": R,
            "t_m": t,
            "T": _transform_from_rt(R, t),
            "position_mm": position_mm,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "reproj_error": reproj_err,
            "tag_size_px": tag_size_px,
            "frame": frame,
        }

    def capture_pose(self, duration_s=0.5, min_frames=10, max_frames=60, frame="robot", reproj_error_max=None):
        """
        Capture multiple frames and return an averaged pose to reduce noise.
        """
        t_start = time.time()
        Rs = []
        ts = []
        errors = []
        tag_sizes = []
        while time.time() - t_start < duration_s and len(Rs) < max_frames:
            pose = self.get_tag_pose(frame=frame)
            if pose is None:
                continue
            if reproj_error_max is not None and pose["reproj_error"] > reproj_error_max:
                continue
            Rs.append(pose["R"])
            ts.append(pose["t_m"])
            errors.append(pose["reproj_error"])
            tag_sizes.append(pose["tag_size_px"])

        if len(Rs) < min_frames:
            return None

        t_avg = np.mean(np.hstack(ts), axis=1, keepdims=True)
        quats = [_quat_from_R(R) for R in Rs]
        q_avg = _average_quaternions(quats)
        R_avg = _R_from_quat(q_avg)

        roll, pitch, yaw = _rotation_to_euler_zyx(R_avg)
        position_mm = (t_avg.flatten() * 1000.0).tolist()

        return {
            "R": R_avg,
            "t_m": t_avg,
            "T": _transform_from_rt(R_avg, t_avg),
            "position_mm": position_mm,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "reproj_error_mean": float(np.mean(errors)),
            "tag_size_px_mean": float(np.mean(tag_sizes)),
            "frames_used": len(Rs),
            "frame": frame,
        }

    def scan_position(self, arm_T_cam, duration_s=1.0, min_frames=15, frame="robot"):
        """
        Capture an averaged tag pose for one robot position and bundle it with arm_T_cam.
        arm_T_cam must be a 4x4 transform (arm->camera).
        """
        arm_T_cam = np.asarray(arm_T_cam, dtype=np.float64)
        if arm_T_cam.shape != (4, 4):
            raise ValueError("arm_T_cam must be a 4x4 transform matrix.")
        tag_pose = self.capture_pose(duration_s=duration_s, min_frames=min_frames, frame=frame)
        if tag_pose is None:
            return None
        return {
            "arm_T_cam": arm_T_cam,
            "tag_T_cam_frame": frame,
            "cam_T_tag": tag_pose["T"],  # tag in camera frame
        }

    def estimate_robot_in_world(self, samples, world_T_tag=None):
        """
        Estimate robot(arm) pose in world using multiple samples.

        samples: list of dicts from scan_position().
        world_T_tag: 4x4 transform (world->tag). If None, assume tag frame == world frame.
        Returns: averaged world_T_arm and per-sample world_T_arm list.
        """
        if world_T_tag is None:
            world_T_tag = np.eye(4, dtype=np.float64)
        else:
            world_T_tag = np.asarray(world_T_tag, dtype=np.float64)
            if world_T_tag.shape != (4, 4):
                raise ValueError("world_T_tag must be a 4x4 transform matrix.")

        R_wt, t_wt = _rt_from_transform(world_T_tag)

        world_T_arms = []
        for s in samples:
            arm_T_cam = np.asarray(s["arm_T_cam"], dtype=np.float64)
            cam_T_tag = np.asarray(s["cam_T_tag"], dtype=np.float64)

            R_ct, t_ct = _rt_from_transform(cam_T_tag)
            R_tc, t_tc = _invert_rt(R_ct, t_ct)

            R_ac, t_ac = _rt_from_transform(arm_T_cam)
            R_ca, t_ca = _invert_rt(R_ac, t_ac)

            # world_T_arm = world_T_tag * tag_T_cam * cam_T_arm
            R_wt_tc, t_wt_tc = _compose_rt(R_wt, t_wt, R_tc, t_tc)
            R_wa, t_wa = _compose_rt(R_wt_tc, t_wt_tc, R_ca, t_ca)
            world_T_arms.append(_transform_from_rt(R_wa, t_wa))

        if not world_T_arms:
            return None

        Rs = [T[:3, :3] for T in world_T_arms]
        ts = [T[:3, 3].reshape(3, 1) for T in world_T_arms]
        t_avg = np.mean(np.hstack(ts), axis=1, keepdims=True)
        quats = [_quat_from_R(R) for R in Rs]
        q_avg = _average_quaternions(quats)
        R_avg = _R_from_quat(q_avg)
        world_T_arm_avg = _transform_from_rt(R_avg, t_avg)

        return {
            "world_T_arm": world_T_arm_avg,
            "world_T_arm_list": world_T_arms,
        }


__all__ = ["AprilTagPoseTracker"]
