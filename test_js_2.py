import time

import cv2
import numpy as np

from apriltag_tracker import AprilTagPoseTracker


def rotation_to_euler_zyx(R):
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

tracker = AprilTagPoseTracker(
      tag_size_mm=31.0,
      tag_family="tagStandard41h12",
      tag_id=0,
      config_file="realsense_D435.yaml",
      resolution="medium",
  )

frame_count = 0
last_print = time.time()
debug_every = 30

print("Starting debug loop. Press Ctrl+C to stop.")
print(f"Tag family: {tracker.tag_family} | Tag ID: {tracker.tag_id}")
print(f"Intrinsics: fx={tracker.camera_matrix[0,0]:.1f}, fy={tracker.camera_matrix[1,1]:.1f}, cx={tracker.camera_matrix[0,2]:.1f}, cy={tracker.camera_matrix[1,2]:.1f}")
print(f"Color format: {tracker.color_format}")

try:
    while True:
        frames = tracker.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        frame_count += 1

        if not color_frame:
            if frame_count % debug_every == 0:
                print("[DBG] No color frame received.")
            continue

        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, tracker.color_to_gray_code)

        detections = tracker.detector.detect(gray)
        if frame_count % debug_every == 0:
            now = time.time()
            fps = debug_every / max(now - last_print, 1e-6)
            last_print = now
            det_ids = [det["id"] for det in detections] if detections else []
            print(f"[DBG] frames={frame_count} fps~{fps:.1f} detections={len(detections)} ids={det_ids}")

        if not detections:
            continue

        target = None
        for det in detections:
            if det["id"] == tracker.tag_id:
                target = det
                break

        if target is None:
            if frame_count % debug_every == 0:
                print("[DBG] Tag detected, but ID does not match target.")
            continue

        corners = tracker._get_corners_from_detection(target)
        corners = tracker._refine_corners_subpix(gray, corners)

        success, rvec, tvec = tracker._estimate_pose(corners)
        if not success:
            print("[DBG] solvePnP failed.")
            continue

        rvec, tvec = tracker._refine_pose(corners, rvec, tvec)
        reproj_err = tracker._compute_reproj_error(corners, rvec, tvec)

        R, _ = cv2.Rodrigues(rvec)
        R, t = tracker._convert_frame(R, tvec, "robot")
        roll, pitch, yaw = rotation_to_euler_zyx(R)
        position_mm = (t.flatten() * 1000.0).tolist()

        print(f"[OK] pos_mm={position_mm} rpy={roll, pitch, yaw} reproj={reproj_err:.2f}")
finally:
    tracker.close()
