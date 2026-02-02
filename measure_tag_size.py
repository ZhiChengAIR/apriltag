#!/usr/bin/env python3
"""
Tag Size Measurement Tool

This script helps you determine the correct tag size by analyzing
the reprojection error at different tag sizes.
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from apriltag import apriltag

def test_tag_sizes(tag_sizes_mm):
    """Test multiple tag sizes and show which gives best reprojection error."""

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

    # Initialize detector
    detector = apriltag("tagStandard41h12")

    print("\nWaiting for stable tag detection...")
    print("Please keep tag STILL in view...\n")

    # Wait for stable frame
    for _ in range(30):
        pipeline.wait_for_frames()

    # Get frame
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # Detect tag
    detections = detector.detect(gray)
    if len(detections) == 0:
        print("ERROR: No tag detected!")
        pipeline.stop()
        return

    # Get corners (reorder from lb-rb-rt-lt to lt-rt-rb-lb)
    raw_corners = detections[0]['lb-rb-rt-lt']
    corners = np.array([
        raw_corners[3],  # lt -> top-left
        raw_corners[2],  # rt -> top-right
        raw_corners[1],  # rb -> bottom-right
        raw_corners[0],  # lb -> bottom-left
    ], dtype=np.float64)

    print("Tag detected! Testing different tag sizes...\n")
    print("="*70)
    print(f"{'Tag Size (mm)':<15} {'Reproj Error (px)':<20} {'Z Distance (mm)':<20}")
    print("="*70)

    best_size = None
    best_error = float('inf')

    for tag_size_mm in tag_sizes_mm:
        tag_size_m = tag_size_mm / 1000.0
        half = tag_size_m / 2.0

        object_points = np.array([
            [-half, -half, 0],
            [ half, -half, 0],
            [ half,  half, 0],
            [-half,  half, 0]
        ], dtype=np.float64)

        success, rvec, tvec = cv2.solvePnP(
            object_points, corners,
            camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if success:
            # Compute reprojection error
            projected, _ = cv2.projectPoints(
                object_points, rvec, tvec,
                camera_matrix, dist_coeffs
            )
            projected = projected.reshape(-1, 2)
            diff = corners - projected
            error = np.sqrt(np.mean(np.sum(diff**2, axis=1)))

            z_distance = tvec[2, 0] * 1000  # mm

            marker = ""
            if error < best_error:
                best_error = error
                best_size = tag_size_mm
                marker = " <-- BEST"

            if error < 1.0:
                marker += " ✓ EXCELLENT"
            elif error < 2.0:
                marker += " ✓ GOOD"

            print(f"{tag_size_mm:<15.1f} {error:<20.2f} {z_distance:<20.1f} {marker}")

    print("="*70)
    print(f"\nRECOMMENDED TAG SIZE: {best_size:.1f} mm")
    print(f"Best reprojection error: {best_error:.2f} pixels")

    if best_error < 1.0:
        print("\n✓ This tag size gives excellent accuracy!")
    elif best_error < 2.0:
        print("\n✓ This tag size gives good accuracy.")
    else:
        print("\n⚠ Warning: Even best tag size has high error.")
        print("  Check:")
        print("    - Tag is printed clearly and flat")
        print("    - Lighting is good")
        print("    - Camera is in focus")

    print(f"\nTo use this tag size, edit test_js_1.py:")
    print(f"  TAG_SIZE_MM = {best_size}")

    pipeline.stop()


if __name__ == "__main__":
    print("="*70)
    print("APRILTAG SIZE MEASUREMENT TOOL")
    print("="*70)
    print("\nThis tool will test different tag sizes to find the correct one.")
    print("\nCommon AprilTag sizes:")
    print("  - 30mm (3.0cm)")
    print("  - 32mm (3.2cm)")
    print("  - 50mm (5.0cm)")
    print("  - 100mm (10.0cm)")
    print("  - 150mm (15.0cm)")

    # Test range of sizes from 20mm to 200mm
    tag_sizes = list(range(20, 201, 5))  # 20, 25, 30, 35, ..., 200

    test_tag_sizes(tag_sizes)
