import time

from apriltag_tracker import AprilTagPoseTracker

tracker = AprilTagPoseTracker(
      tag_size_mm=31.0,
      tag_family="tagStandard41h12",
      tag_id=0,
      config_file="realsense_D435.yaml",
      resolution="medium",
  )

def wait_for_pose(tracker, timeout_s=5.0, frame="robot", warmup_s=0.8, poll_sleep_s=0.01):
    """Wait until a pose is detected or timeout expires."""
    start = time.time()
    warmup_deadline = start + warmup_s

    # Warm up camera/exposure
    while time.time() < warmup_deadline:
        tracker.get_tag_pose(frame=frame)

    # Poll until detected or timeout
    while time.time() - start < timeout_s:
        pose = tracker.get_tag_pose(frame=frame)
        if pose:
            return pose
        if poll_sleep_s:
            time.sleep(poll_sleep_s)

    return None


pose = wait_for_pose(tracker, timeout_s=5.0, frame="robot")
if pose:
    print(pose["position_mm"], pose["roll"], pose["pitch"], pose["yaw"])
else:
    print("No AprilTag detected (timeout).")

tracker.close()
