#!/usr/bin/env python3
"""
Debug script to verify AprilTag corner ordering.
Run this to check if corners are being extracted correctly.
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from apriltag import apriltag

# Initialize RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

# Get camera intrinsics
color_stream = profile.get_stream(rs.stream.color)
intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

camera_matrix = np.array([
    [intrinsics.fx, 0, intrinsics.ppx],
    [0, intrinsics.fy, intrinsics.ppy],
    [0, 0, 1]
], dtype=np.float64)

dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float64)

print(f"Camera Matrix:\n{camera_matrix}")
print(f"Distortion: {dist_coeffs}")

# Initialize detector
detector = apriltag("tagStandard41h12")

# Tag size
TAG_SIZE_M = 0.032  # 32mm
half = TAG_SIZE_M / 2.0

# Object points - verify this ordering!
object_points = np.array([
    [-half, -half, 0],  # 0: top-left
    [ half, -half, 0],  # 1: top-right
    [ half,  half, 0],  # 2: bottom-right
    [-half,  half, 0],  # 3: bottom-left
], dtype=np.float64)

print("\nObject points (3D in meters):")
print("  0 (TL):", object_points[0])
print("  1 (TR):", object_points[1])
print("  2 (BR):", object_points[2])
print("  3 (BL):", object_points[3])

print("\n" + "="*60)
print("Capture a frame with tag visible and press any key...")
print("="*60)

try:
    while True:
        frames = pipeline.wait_for_frames(timeout_ms=1000)
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        detections = detector.detect(gray)

        display = color_image.copy()

        if detections:
            for det in detections:
                if det['id'] == 0:
                    print("\n" + "="*60)
                    print("Detection found! Tag ID:", det['id'])
                    print("="*60)

                    # Print all keys in detection
                    print("\nDetection dictionary keys:", det.keys())

                    # Get raw corners
                    raw = det['lb-rb-rt-lt']
                    print("\nRaw corners from 'lb-rb-rt-lt':")
                    print(f"  [0] lb (left-bottom):  {raw[0]}")
                    print(f"  [1] rb (right-bottom): {raw[1]}")
                    print(f"  [2] rt (right-top):    {raw[2]}")
                    print(f"  [3] lt (left-top):     {raw[3]}")

                    # Draw corners with labels
                    labels = ['lb(0)', 'rb(1)', 'rt(2)', 'lt(3)']
                    colors_raw = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]

                    for i, (pt, label, col) in enumerate(zip(raw, labels, colors_raw)):
                        pt_int = tuple(map(int, pt))
                        cv2.circle(display, pt_int, 8, col, -1)
                        cv2.putText(display, label, (pt_int[0]+10, pt_int[1]),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

                    # Reorder for solvePnP: TL, TR, BR, BL
                    corners = np.array([
                        raw[3],  # lt -> TL
                        raw[2],  # rt -> TR
                        raw[1],  # rb -> BR
                        raw[0],  # lb -> BL
                    ], dtype=np.float64)

                    print("\nReordered corners for solvePnP (TL, TR, BR, BL):")
                    print(f"  [0] TL (was lt[3]): {corners[0]}")
                    print(f"  [1] TR (was rt[2]): {corners[1]}")
                    print(f"  [2] BR (was rb[1]): {corners[2]}")
                    print(f"  [3] BL (was lb[0]): {corners[3]}")

                    # Solve PnP
                    success, rvec, tvec = cv2.solvePnP(
                        object_points, corners,
                        camera_matrix, dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    if success:
                        print(f"\nsolvePnP SUCCESS!")
                        print(f"  tvec (meters): [{tvec[0,0]:.4f}, {tvec[1,0]:.4f}, {tvec[2,0]:.4f}]")
                        print(f"  tvec (mm):     [{tvec[0,0]*1000:.1f}, {tvec[1,0]*1000:.1f}, {tvec[2,0]*1000:.1f}]")

                        # Compute reprojection error
                        projected, _ = cv2.projectPoints(
                            object_points, rvec, tvec,
                            camera_matrix, dist_coeffs
                        )
                        projected = projected.reshape(-1, 2)

                        print("\nReprojection comparison:")
                        total_err = 0
                        for i, (orig, proj) in enumerate(zip(corners, projected)):
                            err = np.linalg.norm(orig - proj)
                            total_err += err**2
                            print(f"  Corner {i}: orig={orig}, proj={proj}, err={err:.2f}px")

                        rms_error = np.sqrt(total_err / 4)
                        print(f"\nRMS Reprojection Error: {rms_error:.3f} pixels")

                        if rms_error < 1.0:
                            print("STATUS: GOOD - Corner mapping is correct!")
                        else:
                            print("STATUS: BAD - Corner mapping may be wrong!")

                        # Draw projected points for comparison
                        for i, proj in enumerate(projected):
                            pt_int = tuple(map(int, proj))
                            cv2.circle(display, pt_int, 4, (255, 0, 255), -1)

                    else:
                        print("\nsolvePnP FAILED!")

                    break

        cv2.putText(display, "Press 'q' to quit, 'd' to capture debug info",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow('Debug Corners', display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('d') and detections:
            print("\nDebug info captured!")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("\nDone.")
