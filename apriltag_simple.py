#!/usr/bin/env python3
"""
Minimal AprilTag pose estimation for RealSense D435.

Frames:
  - OpenCV camera frame:
      X right, Y down, Z forward (out of camera)
  - Robot-style camera frame (optional conversion):
      X right, Y forward/out, Z up

Returned tag pose is "tag in camera frame" (T_tag_cam), consistent with solvePnP:
  X_cam = R * X_tag + t
"""

from __future__ import annotations

from pathlib import Path
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
from apriltag import apriltag


def _load_camera_config(config_file: str | None):
    if not config_file:
        return {"stream": {"width": 640, "height": 480, "fps": 30}}
    config_path = Path(config_file)
    if not config_path.exists():
        alt = Path(__file__).resolve().parent / config_file
        if alt.exists():
            config_path = alt
    if not config_path.exists():
        return {"stream": {"width": 640, "height": 480, "fps": 30}}
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)
    return cfg.get("realsense", {"stream": {"width": 640, "height": 480, "fps": 30}})


def _configure_rgb_sensor(profile, camera_cfg):
    sensor_cfg = camera_cfg.get("rgb_sensor", {}) if camera_cfg else {}
    if not sensor_cfg:
        return
    device = profile.get_device()
    rgb_sensor = None
    for s in device.query_sensors():
        if s.supports(rs.camera_info.name) and "RGB Camera" in s.get_info(rs.camera_info.name):
            rgb_sensor = s
            break
    if rgb_sensor is None:
        return
    param_map = {
        "enable_auto_exposure": rs.option.enable_auto_exposure,
        "enable_auto_white_balance": rs.option.enable_auto_white_balance,
        "exposure": rs.option.exposure,
        "gain": rs.option.gain,
        "white_balance": rs.option.white_balance,
        "brightness": rs.option.brightness,
        "contrast": rs.option.contrast,
        "gamma": rs.option.gamma,
        "saturation": rs.option.saturation,
        "sharpness": rs.option.sharpness,
    }
    for key, opt in param_map.items():
        if key in sensor_cfg and rgb_sensor.supports(opt):
            try:
                rgb_sensor.set_option(opt, sensor_cfg[key])
            except Exception:
                pass


def start_realsense(config_file: str | None = "realsense_D435.yaml", resolution: str | None = None):
    camera_cfg = _load_camera_config(config_file)
    stream_cfg = camera_cfg.get("stream", {})
    width = stream_cfg.get("width", 640)
    height = stream_cfg.get("height", 480)
    fps = stream_cfg.get("fps", 30)
    if resolution:
        preset = {"low": (640, 480), "medium": (1280, 720), "high": (1920, 1080)}
        if resolution not in preset:
            raise ValueError(f"Unknown resolution preset: {resolution}")
        width, height = preset[resolution]

    pipeline = rs.pipeline()
    config = rs.config()
    color_format = rs.format.bgr8
    try:
        config.enable_stream(rs.stream.color, width, height, color_format, fps)
        profile = pipeline.start(config)
        gray_code = cv2.COLOR_BGR2GRAY
    except Exception:
        color_format = rs.format.rgb8
        config.enable_stream(rs.stream.color, width, height, color_format, fps)
        profile = pipeline.start(config)
        gray_code = cv2.COLOR_RGB2GRAY

    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    camera_matrix = np.array([
        [intr.fx, 0, intr.ppx],
        [0, intr.fy, intr.ppy],
        [0, 0, 1],
    ], dtype=np.float64)
    dist_coeffs = np.array(intr.coeffs, dtype=np.float64)

    _configure_rgb_sensor(profile, camera_cfg)

    return {
        "pipeline": pipeline,
        "profile": profile,
        "camera_matrix": camera_matrix,
        "dist_coeffs": dist_coeffs,
        "color_format": color_format,
        "gray_code": gray_code,
    }


def stop_realsense(ctx):
    try:
        ctx["pipeline"].stop()
    except Exception:
        pass


def _object_points(tag_size_mm: float):
    half = (tag_size_mm / 1000.0) / 2.0
    return np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float64)


def _corners_from_detection(det):
    raw = det["lb-rb-rt-lt"]
    return np.array([raw[3], raw[2], raw[1], raw[0]], dtype=np.float64)


def _rotation_to_euler_zyx(R):
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


def _convert_opencv_to_robot(R, t):
    # OpenCV camera frame -> robot-style camera frame
    # x_right stays, y_down -> z_up, z_forward -> y_forward
    T = np.array([
        [1, 0, 0],
        [0, 0, 1],
        [0, -1, 0],
    ], dtype=np.float64)
    return T @ R @ T.T, T @ t


def detect_tag_pose_once(
    ctx,
    tag_family: str,
    tag_size_mm: float,
    tag_id: int | None = 0,
    frame: str = "robot",
):
    pipeline = ctx["pipeline"]
    gray_code = ctx["gray_code"]
    camera_matrix = ctx["camera_matrix"]
    dist_coeffs = ctx["dist_coeffs"]

    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        return None
    color = np.asanyarray(color_frame.get_data())
    gray = cv2.cvtColor(color, gray_code)

    detector = apriltag(tag_family)
    detections = detector.detect(gray)
    if not detections:
        return None

    target = None
    if tag_id is None:
        target = detections[0]
    else:
        for d in detections:
            if int(d["id"]) == int(tag_id):
                target = d
                break
    if target is None:
        return None

    corners = _corners_from_detection(target)
    obj = _object_points(tag_size_mm)

    ok, rvec, tvec = cv2.solvePnP(
        obj, corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    if not ok:
        return None

    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3, 1)

    if frame == "robot":
        R, t = _convert_opencv_to_robot(R, t)
    elif frame != "opencv":
        raise ValueError(f"Unknown frame: {frame}")

    T_tag_cam = np.eye(4, dtype=np.float64)
    T_tag_cam[:3, :3] = R
    T_tag_cam[:3, 3] = t.flatten()

    T_cam_tag = np.linalg.inv(T_tag_cam)
    roll, pitch, yaw = _rotation_to_euler_zyx(R)

    # reprojection error (pixel)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
    proj = proj.reshape(-1, 2)
    err = np.sqrt(np.mean(np.sum((corners - proj) ** 2, axis=1)))

    return {
        "tag_id": int(target["id"]),
        "frame": frame,
        "T_tag_cam": T_tag_cam,
        "T_cam_tag": T_cam_tag,
        "position_mm": (t.flatten() * 1000.0).tolist(),
        "rpy_rad": [float(roll), float(pitch), float(yaw)],
        "reproj_error": float(err),
        "timestamp_s": time.time(),
    }


__all__ = [
    "start_realsense",
    "stop_realsense",
    "detect_tag_pose_once",
]
