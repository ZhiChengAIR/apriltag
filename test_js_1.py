#!/usr/bin/env python3
"""
RealSense D435 AprilTag 6DOF Pose Tracking
Tag: tagStandard41h12 (ID 0)
Real-time tracking at 30+ Hz with pose visualization

COORDINATE SYSTEM (AprilTag Library Convention):
  Camera Frame:
    - Origin: Camera optical center
    - X-axis: Right in image
    - Y-axis: Down in image
    - Z-axis: Forward out of camera lens (depth)

  Tag Frame:
    - Origin: Tag center
    - X-axis: Right (from viewer perspective)
    - Y-axis: Down (from viewer perspective)
    - Z-axis: Into the tag plane

  Output: Tag pose in Camera frame
    - Translation (X, Y, Z) in millimeters
    - Rotation (roll, pitch, yaw) in radians

All units: MILLIMETERS (mm) and RADIANS (rad)
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from apriltag import apriltag
import time
import yaml
import os
from collections import deque
from pathlib import Path
from datetime import datetime


class AprilTagTracker:
    def __init__(self, tag_size_mm=32.0, config_file="realsense_D435.yaml"):
        """
        Initialize AprilTag tracker with RealSense D435

        Args:
            tag_size_mm: AprilTag size in millimeters
            config_file: Path to YAML config file with camera parameters
        """
        self.tag_size_mm = tag_size_mm
        self.tag_size_m = tag_size_mm / 1000.0

        # Load camera configuration from YAML
        self.camera_params = self._load_camera_config(config_file)

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Configure streams from YAML
        stream_config = self.camera_params.get('stream', {})
        width = stream_config.get('width', 640)
        height = stream_config.get('height', 480)
        fps = stream_config.get('fps', 30)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        # Start pipeline
        self.profile = self.pipeline.start(self.config)

        # Get camera intrinsics (factory calibrated)
        color_stream = self.profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

        self.fx = intrinsics.fx
        self.fy = intrinsics.fy
        self.cx = intrinsics.ppx
        self.cy = intrinsics.ppy
        print("fx:", self.fx, "fy:", self.fy, "cx:", self.cx, "cy:", self.cy)
        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float64)

        self.dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float64)

        print("=" * 70)
        print("Camera Intrinsics (Factory Calibrated):")
        print(f"  fx: {self.fx:.2f} px | fy: {self.fy:.2f} px")
        print(f"  cx: {self.cx:.2f} px | cy: {self.cy:.2f} px")
        print(f"  Distortion: {self.dist_coeffs}")
        print("=" * 70)

        # Apply fixed camera parameters from YAML (CRITICAL for stability)
        self._configure_camera_parameters()

        # Frame state for handling dropped frames
        self.last_valid_frame = None
        self.frame_drop_count = 0

        # Initialize AprilTag detector
        self.detector = apriltag("tagStandard41h12")

        # 3D object points for tag corners in TAG coordinate frame
        # Origin at tag center, Z=0 (planar tag)
        half = self.tag_size_m / 2.0
        self.object_points = np.array([
            [-half,  half, 0],  # corner 0: left-top
            [ half,  half, 0],  # corner 1: right-top
            [ half, -half, 0],  # corner 2: right-bottom
            [-half, -half, 0],  # corner 3: left-bottom
        ], dtype=np.float64)

        # FPS tracking
        self.fps_history = deque(maxlen=30)
        self.last_frame_time = time.time()

        # Video recording
        self.is_recording = False
        self.video_writer = None
        self.recording_start_time = None
        self.recording_count = 0

        # Console output throttle
        self.last_print_time = 0

        # UI toggle
        self.show_overlay = True

        # Output directory for snapshots and recordings
        self.output_dir = Path("data")
        self.output_dir.mkdir(exist_ok=True)

    def _load_camera_config(self, config_file):
        """
        Load camera configuration from YAML file.

        Args:
            config_file: Path to YAML config file

        Returns:
            dict: Camera configuration parameters
        """
        config_path = Path(config_file)
        if not config_path.exists():
            print(f"[WARN] Config file not found: {config_file}")
            print("[WARN] Using default parameters (auto-exposure enabled)")
            return {
                'stream': {'width': 640, 'height': 480, 'fps': 30},
                'rgb_sensor': {'enable_auto_exposure': 1}
            }

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        if 'realsense' not in config:
            print(f"[WARN] Invalid config format in {config_file}")
            return {'stream': {'width': 640, 'height': 480, 'fps': 30}}

        return config['realsense']

    def _find_rgb_sensor(self):
        """
        Find the RGB camera sensor by name (NOT by index).

        Returns:
            rs.sensor: RGB sensor object, or None if not found
        """
        device = self.profile.get_device()
        for sensor in device.query_sensors():
            if sensor.supports(rs.camera_info.name):
                sensor_name = sensor.get_info(rs.camera_info.name)
                if "RGB Camera" in sensor_name:
                    return sensor
        return None

    def _configure_camera_parameters(self):
        """
        Apply fixed camera parameters from YAML config.
        CRITICAL: Disables auto-exposure and auto-white-balance to eliminate pose jitter.

        Parameter application order:
          1. Disable auto-exposure
          2. Disable auto-white-balance
          3. Set fixed exposure, gain, white_balance
          4. Set optional parameters (brightness, contrast, etc.)
        """
        # Find RGB sensor by name (not by index - this was a bug)
        rgb_sensor = self._find_rgb_sensor()
        if rgb_sensor is None:
            print("[ERROR] Could not find RGB Camera sensor!")
            print("[WARN] Camera parameters NOT applied - expect pose jitter!")
            return

        sensor_params = self.camera_params.get('rgb_sensor', {})
        if not sensor_params:
            print("[WARN] No rgb_sensor config found in YAML")
            return

        print("\nApplying Camera Parameters:")
        print("-" * 70)

        # Parameter mapping: YAML key -> RealSense option -> description
        param_map = {
            'enable_auto_exposure': (rs.option.enable_auto_exposure, "Auto Exposure"),
            'enable_auto_white_balance': (rs.option.enable_auto_white_balance, "Auto White Balance"),
            'exposure': (rs.option.exposure, "Exposure"),
            'gain': (rs.option.gain, "Gain"),
            'white_balance': (rs.option.white_balance, "White Balance"),
            'brightness': (rs.option.brightness, "Brightness"),
            'contrast': (rs.option.contrast, "Contrast"),
            'gamma': (rs.option.gamma, "Gamma"),
            'saturation': (rs.option.saturation, "Saturation"),
            'sharpness': (rs.option.sharpness, "Sharpness"),
        }

        # Apply parameters in order
        for yaml_key, (rs_option, description) in param_map.items():
            if yaml_key in sensor_params:
                value = sensor_params[yaml_key]
                if rgb_sensor.supports(rs_option):
                    try:
                        rgb_sensor.set_option(rs_option, value)
                        print(f"  {description:20s} = {value}")
                    except Exception as e:
                        print(f"  [ERROR] Failed to set {description}: {e}")
                else:
                    print(f"  [SKIP] {description} not supported by this sensor")

        print("-" * 70)

    def get_corners_from_detection(self, detection):
        """
        Extract corners from detection in correct order for solvePnP.
        AprilTag returns corners as 'lb-rb-rt-lt' (left-bottom, right-bottom, right-top, left-top)

        Order to match object_points: [left-top, right-top, right-bottom, left-bottom]
        """
        raw_corners = detection['lb-rb-rt-lt']
        # raw_corners order: [lb, rb, rt, lt] = [0, 1, 2, 3]
        # We need: [lt, rt, rb, lb] = [3, 2, 1, 0]
        corners = np.array([
            raw_corners[3],  # lt -> left-top
            raw_corners[2],  # rt -> right-top
            raw_corners[1],  # rb -> right-bottom
            raw_corners[0],  # lb -> left-bottom
        ], dtype=np.float64)
        return corners

    def estimate_pose(self, corners):
        """
        Estimate pose using solvePnP.
        Returns (success, rvec, tvec) where tvec is in METERS.
        """
        success, rvec, tvec = cv2.solvePnP(
            self.object_points,
            corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        return success, rvec, tvec

    def compute_reprojection_error(self, corners, rvec, tvec):
        """Compute RMS reprojection error in pixels."""
        projected, _ = cv2.projectPoints(
            self.object_points, rvec, tvec,
            self.camera_matrix, self.dist_coeffs
        )
        projected = projected.reshape(-1, 2)
        diff = corners - projected
        error = np.sqrt(np.mean(np.sum(diff**2, axis=1)))
        return error

    def rotation_to_euler(self, R):
        """
        Convert rotation matrix to Euler angles (roll, pitch, yaw).
        Convention: ZYX (yaw-pitch-roll)
        """
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
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

    def convert_to_left_handed_camera_pose(self, rvec, tvec):
        """
        Convert from tag-in-camera to camera-in-tag (left-handed).

        CRITICAL POSE INVERSION:
          solvePnP returns: T_tag_in_cam (where is tag relative to camera?)
          We output: T_cam_in_tag (where is camera relative to tag?)

          Why? The AprilTag is a FIXED world reference point.
               The camera is MOVING (mounted on robot).
               For robot control, we need: "where is my camera/robot relative to the tag?"

        COORDINATE SYSTEM TRANSFORM:
          OpenCV (right-handed): X=right, Y=down, Z=forward (toward camera)
          Output (left-handed):  X=right, Y=depth (away from tag), Z=up

          Mapping: X_out = X_cv, Y_out = Z_cv, Z_out = -Y_cv

        Args:
            rvec: Rotation vector from solvePnP (tag in camera frame)
            tvec: Translation vector from solvePnP (tag in camera frame)

        Returns:
            tuple: (x_mm, y_mm, z_mm, roll, pitch, yaw, R_left)
                   Position in mm, orientation in radians, rotation matrix
        """
        # Step 1: Get rotation matrix from rvec (tag pose in camera frame)
        R_tag_in_cam, _ = cv2.Rodrigues(rvec)
        t_tag_in_cam = tvec.flatten()

        # Step 2: INVERT to get camera pose in tag frame
        # This is the KEY operation - we want to know where the CAMERA is, not where the TAG is
        # Mathematical inversion of SE(3) transform:
        #   R_inv = R^T
        #   t_inv = -R^T * t
        R_cam_in_tag = R_tag_in_cam.T
        t_cam_in_tag = -R_cam_in_tag @ t_tag_in_cam

        # Step 3: Convert from OpenCV (right-handed) to left-handed coordinates
        # OpenCV tag frame (right-handed): X=right, Y=down, Z=forward (toward camera)
        # Application frame (left-handed): X=right, Y=depth (away from tag), Z=up
        #
        # Coordinate mapping:
        #   X_left = X_opencv          (right is right)
        #   Y_left = Z_opencv          (depth = opencv's Z)
        #   Z_left = -Y_opencv         (up = negative of opencv's down)
        #
        # Transform matrix T implements this mapping
        T = np.array([
            [1,  0,  0],  # X_left = X_opencv
            [0,  0,  1],  # Y_left = Z_opencv
            [0, -1,  0]   # Z_left = -Y_opencv
        ], dtype=np.float64)

        # Apply coordinate transform to translation
        t_left = T @ t_cam_in_tag

        # Apply coordinate transform to rotation (similarity transform: T R T^T)
        R_left = T @ R_cam_in_tag @ T.T

        # Convert to mm
        x_mm = t_left[0] * 1000.0
        y_mm = t_left[1] * 1000.0
        z_mm = t_left[2] * 1000.0

        # Get Euler angles
        roll, pitch, yaw = self.rotation_to_euler(R_left)

        return x_mm, y_mm, z_mm, roll, pitch, yaw, R_left

    def process_frame(self, color_image):
        """
        Process frame and return pose data.
        """
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        if not detections:
            return None

        # Find tag ID 0
        target_det = None
        for det in detections:
            if det['id'] == 0:
                target_det = det
                break

        if target_det is None:
            return None

        # Get corners
        corners = self.get_corners_from_detection(target_det)

        # Estimate pose
        success, rvec, tvec = self.estimate_pose(corners)
        if not success:
            return None

        # Compute reprojection error
        reproj_err = self.compute_reprojection_error(corners, rvec, tvec)

        # SIMPLE: Just use the raw solvePnP output in millimeters
        # solvePnP returns tag position in camera frame (meters)
        # Convert to millimeters directly
        x_mm = tvec[0, 0] * 1000.0
        y_mm = tvec[1, 0] * 1000.0
        z_mm = tvec[2, 0] * 1000.0

        # Get rotation matrix
        R, _ = cv2.Rodrigues(rvec)
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
        yaw = np.arctan2(R[1, 0], R[0, 0])

        return {
            'x_mm': x_mm,
            'y_mm': y_mm,
            'z_mm': z_mm,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'reproj_error': reproj_err,
            'corners': corners,
            'rvec': rvec,
            'tvec': tvec,
            'R_left': R,
            'tag_id': target_det['id']
        }

    def draw_tag_axes(self, image, rvec, tvec):
        """Draw 3D coordinate axes on the tag."""
        axis_length = self.tag_size_m * 0.75

        axis_points = np.array([
            [0, 0, 0],
            [axis_length, 0, 0],
            [0, axis_length, 0],
            [0, 0, -axis_length]  # Z points out of tag (toward camera)
        ], dtype=np.float64)

        img_pts, _ = cv2.projectPoints(
            axis_points, rvec, tvec,
            self.camera_matrix, self.dist_coeffs
        )
        img_pts = img_pts.reshape(-1, 2).astype(int)

        origin = tuple(img_pts[0])
        cv2.line(image, origin, tuple(img_pts[1]), (0, 0, 255), 3)  # X: Red
        cv2.line(image, origin, tuple(img_pts[2]), (0, 255, 0), 3)  # Y: Green
        cv2.line(image, origin, tuple(img_pts[3]), (255, 0, 0), 3)  # Z: Blue

        # Labels
        cv2.putText(image, 'X', tuple(img_pts[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(image, 'Y', tuple(img_pts[2]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(image, 'Z', tuple(img_pts[3]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    def draw_tag_outline(self, image, corners):
        """Draw tag outline with corner numbers."""
        pts = corners.astype(int)
        for i in range(4):
            cv2.line(image, tuple(pts[i]), tuple(pts[(i + 1) % 4]), (0, 255, 0), 2)

        # Label each corner
        colors = [(255, 0, 0), (0, 255, 0), (255, 255, 0), (255, 0, 255)]  # Different color per corner
        for i in range(4):
            cv2.circle(image, tuple(pts[i]), 8, colors[i], -1)
            cv2.putText(image, str(i), (pts[i][0] + 10, pts[i][1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[i], 2)


    def draw_ui(self, image, pose_data, fps, frame_dropped=False):
        """
        Draw simplified UI overlay.

        Args:
            image: Frame to draw on
            pose_data: Detected pose data (or None)
            fps: Current FPS
            frame_dropped: Whether the current frame was dropped
        """
        h, w = image.shape[:2]

        if not self.show_overlay:
            return image

        if pose_data is not None:
            # Draw tag visualization
            self.draw_tag_outline(image, pose_data['corners'])
            self.draw_tag_axes(image, pose_data['rvec'], pose_data['tvec'])

            # Simplified info panel
            cv2.rectangle(image, (10, 10), (280, 180), (20, 20, 20), -1)
            cv2.rectangle(image, (10, 10), (280, 180), (80, 80, 80), 1)

            # Title
            cv2.putText(image, "Tag Position (Camera Frame)", (20, 35),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # Position
            cv2.putText(image, "Position (mm):", (20, 65),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(image, f"  X: {pose_data['x_mm']:8.1f}", (20, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 255), 1)
            cv2.putText(image, f"  Y: {pose_data['y_mm']:8.1f}", (20, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 100), 1)
            cv2.putText(image, f"  Z: {pose_data['z_mm']:8.1f}", (20, 130),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 100, 100), 1)

            # Orientation
            cv2.putText(image, "Orientation (rad):", (20, 155),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(image, f"  R: {pose_data['roll']:+.3f}  P: {pose_data['pitch']:+.3f}  Y: {pose_data['yaw']:+.3f}",
                       (20, 175),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        else:
            # No tag detected - show message on camera stream
            cv2.putText(image, "NO TAG DETECTED", (20, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # FPS display
        cv2.rectangle(image, (w - 100, 10), (w - 10, 45), (20, 20, 20), -1)
        fps_color = (0, 150, 255) if frame_dropped else (0, 255, 0)
        cv2.putText(image, f"FPS: {fps:.1f}", (w - 95, 35),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, fps_color, 1)

        # Frame drop indicator
        if frame_dropped:
            cv2.putText(image, "DROP", (w - 90, h - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 150, 255), 1)

        # Recording indicator
        if self.is_recording:
            elapsed = time.time() - self.recording_start_time
            cv2.circle(image, (w - 120, 27), 8, (0, 0, 255), -1)
            cv2.putText(image, f"REC {elapsed:.0f}s", (w - 180, 35),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        # Controls help
        cv2.putText(image, "o:Display  c:Recording  s:StopRec  x:Snapshot  Files->data/", (10, h - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)

        return image

    def start_recording(self, w, h):
        """Start video recording."""
        if self.is_recording:
            return
        filename = self.output_dir / f"recording_{self.recording_count:03d}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(str(filename), fourcc, 30.0, (w, h))
        self.is_recording = True
        self.recording_start_time = time.time()
        print(f"\n[REC START] Saving to: {filename}")

    def stop_recording(self):
        """Stop video recording."""
        if not self.is_recording:
            return
        self.is_recording = False
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        dur = time.time() - self.recording_start_time
        print(f"\n[REC STOP] Duration: {dur:.1f}s | Saved to: data/")
        self.recording_count += 1

    def run(self):
        """
        Main tracking loop.

        STABILITY FEATURES:
          - Non-blocking frame acquisition prevents UI freezes
          - Last frame reuse when frames are dropped
          - Fixed camera parameters (no auto-exposure jitter)
          - Throttled console output (low overhead)
        """
        print("\n" + "=" * 70)
        print("AprilTag 6DOF Pose Tracker - RealSense D435")
        print("=" * 70)
        print(f"Tag: tagStandard41h12 | ID: 0 | Size: {self.tag_size_mm} mm")
        print("\nCOORDINATE SYSTEM (AprilTag Library Convention):")
        print("  Output: Tag pose in Camera frame")
        print("  X: Right(+) in image")
        print("  Y: Down(+) in image")
        print("  Z: Forward(+) = depth from camera")
        print("\nUNITS: millimeters (mm) and radians (rad)")
        print(f"\nOUTPUT: All files saved to: {self.output_dir}/")
        print("\nControls:")
        print("  o = Toggle display overlay")
        print("  c = Start recording")
        print("  s = Stop recording")
        print("  x = Snapshot")
        print("  d = Debug info")
        print("  q = Quit")
        print("=" * 70)
        print("\nLive pose:\n")

        try:
            while True:
                # NON-BLOCKING frame acquisition with fallback
                # Try poll_for_frames() first (non-blocking)
                frames = self.pipeline.poll_for_frames()

                frame_dropped = False
                color_image = None

                # Check if we got valid frames
                if frames and frames.get_color_frame():
                    # Got a new frame
                    color_frame = frames.get_color_frame()
                    color_image = np.asanyarray(color_frame.get_data())
                    self.last_valid_frame = color_image.copy()
                    self.frame_drop_count = 0
                else:
                    # No frame available from poll - try wait with short timeout
                    # This prevents spinning at 1000+ FPS and gives camera time to produce frames
                    try:
                        frames = self.pipeline.wait_for_frames(timeout_ms=100)
                        color_frame = frames.get_color_frame()
                        if color_frame:
                            color_image = np.asanyarray(color_frame.get_data())
                            self.last_valid_frame = color_image.copy()
                            self.frame_drop_count = 0
                        else:
                            frame_dropped = True
                            self.frame_drop_count += 1
                    except RuntimeError:
                        # Timeout - reuse last valid frame
                        frame_dropped = True
                        self.frame_drop_count += 1

                    if color_image is None:
                        if self.last_valid_frame is not None:
                            # Reuse last frame for display (keeps UI responsive)
                            color_image = self.last_valid_frame
                        else:
                            # No frames yet - show blank screen with message
                            color_image = np.zeros((480, 640, 3), dtype=np.uint8)
                            cv2.putText(color_image, "Waiting for camera...", (180, 240),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                # Process frame (only if we have valid image data)
                pose_data = None
                if color_image is not None and not frame_dropped:
                    pose_data = self.process_frame(color_image)

                # Calculate FPS (based on UI loop, not camera framerate)
                now = time.time()
                dt = now - self.last_frame_time
                self.last_frame_time = now
                if dt > 0:
                    self.fps_history.append(1.0 / dt)
                fps = np.mean(self.fps_history) if self.fps_history else 0

                # Draw UI
                display = self.draw_ui(color_image.copy(), pose_data, fps, frame_dropped)

                # Console output (throttled to ~2Hz to reduce overhead)
                if now - self.last_print_time > 0.5:
                    self.last_print_time = now
                    if pose_data:
                        print(f"\r[FPS:{fps:4.1f}] X:{pose_data['x_mm']:7.1f} Y:{pose_data['y_mm']:7.1f} Z:{pose_data['z_mm']:7.1f} mm | "
                              f"R:{pose_data['roll']:+5.3f} P:{pose_data['pitch']:+5.3f} Y:{pose_data['yaw']:+5.3f} rad | "
                              f"err:{pose_data['reproj_error']:.2f}px", end='', flush=True)
                    elif frame_dropped:
                        print(f"\r[FPS:{fps:4.1f}] [FRAME DROPPED: {self.frame_drop_count}]" + " " * 60, end='', flush=True)
                    else:
                        print(f"\r[FPS:{fps:4.1f}] [NO TAG]" + " " * 80, end='', flush=True)

                # Record if active (only record non-dropped frames to avoid duplicates)
                if self.is_recording and self.video_writer and not frame_dropped:
                    self.video_writer.write(display)

                # Show (this will NEVER freeze now thanks to poll_for_frames)
                cv2.imshow('AprilTag Tracker', display)

                # Handle keys
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('o'):
                    # Toggle overlay display
                    self.show_overlay = not self.show_overlay
                    print(f"\n[UI] Overlay: {'ON' if self.show_overlay else 'OFF'}")
                elif key == ord('x'):
                    # Snapshot with datetime
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    fn = self.output_dir / f"snapshot_{timestamp}.png"
                    cv2.imwrite(str(fn), display)
                    print(f"\n[SNAPSHOT] Saved: {fn}")
                elif key == ord('d'):
                    # Debug: comprehensive diagnostics
                    if pose_data:
                        print("\n" + "="*70)
                        print("DIAGNOSTIC OUTPUT")
                        print("="*70)
                        print(f"Tag size configured: {self.tag_size_mm} mm")
                        print(f"\nCorner positions (image pixels):")
                        corners = pose_data['corners']
                        for i, corner in enumerate(corners):
                            print(f"  Corner {i}: ({corner[0]:.1f}, {corner[1]:.1f})")

                        # Calculate tag size in image
                        p0, p1, p2, p3 = corners
                        width_top = np.linalg.norm(p1 - p0)
                        width_bottom = np.linalg.norm(p2 - p3)
                        height_left = np.linalg.norm(p3 - p0)
                        height_right = np.linalg.norm(p2 - p1)
                        avg_size_px = (width_top + width_bottom + height_left + height_right) / 4

                        print(f"\nTag size in image:")
                        print(f"  Top edge: {width_top:.1f} px")
                        print(f"  Bottom edge: {width_bottom:.1f} px")
                        print(f"  Left edge: {height_left:.1f} px")
                        print(f"  Right edge: {height_right:.1f} px")
                        print(f"  Average: {avg_size_px:.1f} px")

                        print(f"\nPose estimation:")
                        print(f"  Position (mm): X={pose_data['x_mm']:.1f}, Y={pose_data['y_mm']:.1f}, Z={pose_data['z_mm']:.1f}")
                        print(f"  Rotation (rad): R={pose_data['roll']:.3f}, P={pose_data['pitch']:.3f}, Y={pose_data['yaw']:.3f}")
                        print(f"  Reprojection error: {pose_data['reproj_error']:.1f} pixels")

                        if pose_data['reproj_error'] > 10:
                            print(f"\n⚠ WARNING: Reprojection error is very high!")
                            print(f"  Possible causes:")
                            print(f"    1. Wrong tag size (check if tag is actually {self.tag_size_mm}mm)")
                            print(f"    2. Corner ordering mismatch")
                            print(f"    3. Run: python3 measure_tag_size.py")
                        print("="*70)
                elif key == ord('c'):
                    # Start recording
                    h, w = display.shape[:2]
                    self.start_recording(w, h)
                elif key == ord('s'):
                    # Stop recording
                    self.stop_recording()

        except KeyboardInterrupt:
            print("\n\nInterrupted.")
        finally:
            if self.is_recording:
                self.stop_recording()
            self.pipeline.stop()
            cv2.destroyAllWindows()
            print("\nTracker stopped.")


if __name__ == "__main__":
    TAG_SIZE_MM = 31.0  # Your tag size: 3.1 cm = 31 mm
    tracker = AprilTagTracker(tag_size_mm=TAG_SIZE_MM)
    tracker.run()
