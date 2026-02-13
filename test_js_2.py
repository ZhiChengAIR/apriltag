from apriltag_tracker import AprilTagPoseTracker

tracker = AprilTagPoseTracker(
      tag_size_mm=31.0,
      tag_family="tagStandard41h12",
      tag_id=0,
      config_file="realsense_D435.yaml",
      resolution="medium",
  )

try:
    while True:
        pose = tracker.get_tag_pose(frame="robot")
        if pose:
            print(pose["position_mm"], pose["roll"], pose["pitch"], pose["yaw"])
finally:
    tracker.close()
