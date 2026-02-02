#!/usr/bin/env python3
"""
AprilTag Pose Accuracy Test Script

This script helps you measure and validate the accuracy of AprilTag pose estimation.

Test Methods:
1. Distance Test: Measure actual distance vs detected distance
2. Translation Test: Move tag by known amount, measure detected movement
3. Rotation Test: Rotate tag by known angle, measure detected rotation
4. Stability Test: Keep tag static, measure pose variance (jitter)
5. Repeatability Test: Return to same position multiple times
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from apriltag import apriltag
import time


class AccuracyTester:
    def __init__(self, tag_size_mm=32.0):
        self.tag_size_mm = tag_size_mm
        self.tag_size_m = tag_size_mm / 1000.0

        # Initialize RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline.start(self.config)

        # Get camera intrinsics
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

        self.camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float64)

        self.dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float64)

        print("\nCamera Intrinsics:")
        print(f"  fx: {intrinsics.fx:.4f} pixels")
        print(f"  fy: {intrinsics.fy:.4f} pixels")
        print(f"  cx: {intrinsics.ppx:.4f} pixels")
        print(f"  cy: {intrinsics.ppy:.4f} pixels")
        print(f"  Distortion: {self.dist_coeffs}")

        # Initialize detector
        self.detector = apriltag("tagStandard41h12")

        # 3D tag corners
        half_size = self.tag_size_m / 2.0
        self.object_points = np.array([
            [-half_size, -half_size, 0],
            [half_size, -half_size, 0],
            [half_size, half_size, 0],
            [-half_size, half_size, 0]
        ], dtype=np.float64)

    def detect_pose(self):
        """Detect tag and return pose"""
        frames = self.pipeline.wait_for_frames(timeout_ms=5000)
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None

        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        if len(detections) == 0:
            return None

        detection = None
        for det in detections:
            if det['id'] == 0:
                detection = det
                break

        if detection is None:
            return None

        corners = np.array([
            detection['lb-rb-rt-lt'][3],
            detection['lb-rb-rt-lt'][2],
            detection['lb-rb-rt-lt'][1],
            detection['lb-rb-rt-lt'][0]
        ], dtype=np.float64)

        success, rvec, tvec = cv2.solvePnP(
            self.object_points, corners,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if not success:
            return None

        R, _ = cv2.Rodrigues(rvec)
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
        yaw = np.arctan2(R[1, 0], R[0, 0])

        # Reprojection error
        projected, _ = cv2.projectPoints(self.object_points, rvec, tvec,
                                        self.camera_matrix, self.dist_coeffs)
        projected = projected.reshape(-1, 2)
        reproj_error = np.sqrt(np.sum((corners - projected)**2) / len(corners))

        return {
            'x_mm': tvec[0, 0] * 1000,
            'y_mm': tvec[1, 0] * 1000,
            'z_mm': tvec[2, 0] * 1000,
            'roll_rad': roll,
            'pitch_rad': pitch,
            'yaw_rad': yaw,
            'reproj_error': reproj_error,
            'image': color_image
        }

    def test_distance_accuracy(self):
        """
        Test 1: Distance Accuracy
        Measure tag at known distances and compare
        """
        print("\n" + "="*70)
        print("TEST 1: DISTANCE ACCURACY")
        print("="*70)
        print("\nInstructions:")
        print("1. Place tag at a known distance (e.g., 300mm)")
        print("2. Use a ruler/caliper to measure actual distance")
        print("3. Press 'm' to measure, enter actual distance")
        print("4. Repeat for different distances (e.g., 200mm, 300mm, 400mm)")
        print("5. Press 'q' to finish\n")

        measurements = []

        while True:
            pose = self.detect_pose()

            if pose is not None:
                cv2.putText(pose['image'], f"Z: {pose['z_mm']:.2f} mm", (20, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(pose['image'], "Press 'm' to measure", (20, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.imshow('Distance Test', pose['image'])
            else:
                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(blank, "NO TAG DETECTED", (150, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow('Distance Test', blank)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('m') and pose is not None:
                detected = pose['z_mm']
                cv2.destroyAllWindows()
                actual = float(input(f"\nDetected: {detected:.2f} mm | Enter actual distance (mm): "))
                error = detected - actual
                error_pct = (error / actual) * 100
                measurements.append({
                    'actual': actual,
                    'detected': detected,
                    'error': error,
                    'error_pct': error_pct
                })
                print(f"Error: {error:+.2f} mm ({error_pct:+.2f}%)")
                print(f"Measurements so far: {len(measurements)}\n")

        cv2.destroyAllWindows()

        if len(measurements) > 0:
            print("\n" + "-"*70)
            print("DISTANCE TEST RESULTS:")
            print("-"*70)
            for i, m in enumerate(measurements):
                print(f"{i+1}. Actual: {m['actual']:6.1f} mm | Detected: {m['detected']:6.1f} mm | "
                      f"Error: {m['error']:+6.2f} mm ({m['error_pct']:+5.2f}%)")

            errors = [abs(m['error']) for m in measurements]
            print(f"\nMean Absolute Error: {np.mean(errors):.2f} mm")
            print(f"Std Dev: {np.std(errors):.2f} mm")
            print(f"Max Error: {np.max(errors):.2f} mm")

            return measurements
        return None

    def test_stability(self, duration=10):
        """
        Test 2: Stability Test
        Keep tag static and measure pose variance (jitter)
        """
        print("\n" + "="*70)
        print("TEST 2: STABILITY TEST")
        print("="*70)
        print(f"\nInstructions:")
        print(f"1. Place tag in a stable position")
        print(f"2. Press 's' to start {duration}s measurement")
        print(f"3. DO NOT MOVE the tag during measurement")
        print(f"4. Press 'q' to skip\n")

        while True:
            pose = self.detect_pose()

            if pose is not None:
                cv2.putText(pose['image'], "Press 's' to start stability test", (20, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow('Stability Test', pose['image'])
            else:
                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(blank, "NO TAG DETECTED", (150, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow('Stability Test', blank)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
                return None
            elif key == ord('s') and pose is not None:
                break

        # Collect measurements
        print(f"\nMeasuring for {duration} seconds... DO NOT MOVE TAG!")
        measurements = []
        start_time = time.time()

        while time.time() - start_time < duration:
            pose = self.detect_pose()
            if pose is not None:
                measurements.append(pose)
                elapsed = time.time() - start_time
                cv2.putText(pose['image'], f"Measuring: {elapsed:.1f}s / {duration}s",
                           (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(pose['image'], f"Samples: {len(measurements)}",
                           (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.imshow('Stability Test', pose['image'])
                cv2.waitKey(1)

        cv2.destroyAllWindows()

        if len(measurements) > 0:
            x_vals = [m['x_mm'] for m in measurements]
            y_vals = [m['y_mm'] for m in measurements]
            z_vals = [m['z_mm'] for m in measurements]
            roll_vals = [m['roll_rad'] for m in measurements]
            pitch_vals = [m['pitch_rad'] for m in measurements]
            yaw_vals = [m['yaw_rad'] for m in measurements]

            print("\n" + "-"*70)
            print("STABILITY TEST RESULTS:")
            print("-"*70)
            print(f"Samples collected: {len(measurements)}")
            print(f"Duration: {duration} seconds")
            print(f"Sample rate: {len(measurements)/duration:.1f} Hz\n")

            print("Position Variance (mm):")
            print(f"  X: mean={np.mean(x_vals):7.3f}, std={np.std(x_vals):.4f}, range={np.ptp(x_vals):.4f}")
            print(f"  Y: mean={np.mean(y_vals):7.3f}, std={np.std(y_vals):.4f}, range={np.ptp(y_vals):.4f}")
            print(f"  Z: mean={np.mean(z_vals):7.3f}, std={np.std(z_vals):.4f}, range={np.ptp(z_vals):.4f}")

            print("\nOrientation Variance (rad):")
            print(f"  Roll:  mean={np.mean(roll_vals):7.4f}, std={np.std(roll_vals):.5f}, range={np.ptp(roll_vals):.5f}")
            print(f"  Pitch: mean={np.mean(pitch_vals):7.4f}, std={np.std(pitch_vals):.5f}, range={np.ptp(pitch_vals):.5f}")
            print(f"  Yaw:   mean={np.mean(yaw_vals):7.4f}, std={np.std(yaw_vals):.5f}, range={np.ptp(yaw_vals):.5f}")

            reproj_errors = [m['reproj_error'] for m in measurements]
            print(f"\nReprojection Error: mean={np.mean(reproj_errors):.4f} px, max={np.max(reproj_errors):.4f} px")

            # Assessment
            print("\n" + "-"*70)
            print("ASSESSMENT:")
            pos_std = np.mean([np.std(x_vals), np.std(y_vals), np.std(z_vals)])
            rot_std = np.mean([np.std(roll_vals), np.std(pitch_vals), np.std(yaw_vals)])

            if pos_std < 0.5:
                print(f"Position stability: EXCELLENT (std < 0.5 mm)")
            elif pos_std < 1.0:
                print(f"Position stability: GOOD (std < 1.0 mm)")
            elif pos_std < 2.0:
                print(f"Position stability: FAIR (std < 2.0 mm)")
            else:
                print(f"Position stability: POOR (std >= 2.0 mm)")

            if rot_std < 0.001:
                print(f"Rotation stability: EXCELLENT (std < 0.001 rad)")
            elif rot_std < 0.005:
                print(f"Rotation stability: GOOD (std < 0.005 rad)")
            elif rot_std < 0.01:
                print(f"Rotation stability: FAIR (std < 0.01 rad)")
            else:
                print(f"Rotation stability: POOR (std >= 0.01 rad)")

            return measurements
        return None

    def test_translation_accuracy(self):
        """
        Test 3: Translation Accuracy
        Move tag by known distance, measure detected movement
        """
        print("\n" + "="*70)
        print("TEST 3: TRANSLATION ACCURACY")
        print("="*70)
        print("\nInstructions:")
        print("1. Place tag at starting position")
        print("2. Press 'a' to record position A")
        print("3. Move tag by a known distance (e.g., 50mm horizontally)")
        print("4. Press 'b' to record position B")
        print("5. Enter actual movement distance")
        print("6. Press 'q' to finish\n")

        position_a = None
        position_b = None

        while True:
            pose = self.detect_pose()

            if pose is not None:
                status = "Waiting for position A..." if position_a is None else "Waiting for position B..."
                cv2.putText(pose['image'], status, (20, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                if position_a is None:
                    cv2.putText(pose['image'], "Press 'a' to capture position A", (20, 80),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                else:
                    cv2.putText(pose['image'], "Press 'b' to capture position B", (20, 80),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.imshow('Translation Test', pose['image'])
            else:
                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(blank, "NO TAG DETECTED", (150, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow('Translation Test', blank)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('a') and pose is not None and position_a is None:
                position_a = pose
                print(f"\nPosition A captured: X={pose['x_mm']:.2f}, Y={pose['y_mm']:.2f}, Z={pose['z_mm']:.2f}")
            elif key == ord('b') and pose is not None and position_a is not None:
                position_b = pose
                print(f"Position B captured: X={pose['x_mm']:.2f}, Y={pose['y_mm']:.2f}, Z={pose['z_mm']:.2f}")

                dx = position_b['x_mm'] - position_a['x_mm']
                dy = position_b['y_mm'] - position_a['y_mm']
                dz = position_b['z_mm'] - position_a['z_mm']
                detected_dist = np.sqrt(dx**2 + dy**2 + dz**2)

                cv2.destroyAllWindows()
                print(f"\nDetected movement: {detected_dist:.2f} mm")
                print(f"  dX: {dx:+.2f} mm")
                print(f"  dY: {dy:+.2f} mm")
                print(f"  dZ: {dz:+.2f} mm")

                actual = float(input("\nEnter actual movement distance (mm): "))
                error = detected_dist - actual
                error_pct = (error / actual) * 100 if actual > 0 else 0

                print(f"\nTranslation Error: {error:+.2f} mm ({error_pct:+.2f}%)")

                position_a = None
                position_b = None

        cv2.destroyAllWindows()

    def run_all_tests(self):
        """Run all accuracy tests"""
        print("\n" + "="*70)
        print("APRILTAG ACCURACY TEST SUITE")
        print("="*70)
        print(f"\nTag Size: {self.tag_size_mm} mm")
        print("Tag Family: tagStandard41h12")
        print("Tag ID: 0")

        try:
            # Test 1: Distance
            self.test_distance_accuracy()

            # Test 2: Stability
            self.test_stability(duration=10)

            # Test 3: Translation
            self.test_translation_accuracy()

            print("\n" + "="*70)
            print("ALL TESTS COMPLETED")
            print("="*70)

        except KeyboardInterrupt:
            print("\n\nTests interrupted by user")
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    tester = AccuracyTester(tag_size_mm=32.0)
    tester.run_all_tests()
