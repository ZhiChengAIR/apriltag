#!/usr/bin/env python3
"""
Simple AprilTag pose readout (single frame by default).

Outputs:
  - T_tag_cam (tag in camera frame)
  - position_mm (tag origin in camera frame)
  - rpy_rad (roll, pitch, yaw, ZYX)
"""

import argparse
import sys
from pathlib import Path

# Ensure local apriltag utilities are on path
sys.path.insert(0, str(Path(__file__).resolve().parent))

from apriltag_simple import start_realsense, stop_realsense, detect_tag_pose_once


def main():
    parser = argparse.ArgumentParser(description="AprilTag pose (single-shot)")
    parser.add_argument("--tag-family", type=str, default="tagStandard41h12")
    parser.add_argument("--tag-size", type=float, default=31.0, help="Tag size in mm")
    parser.add_argument("--tag-id", type=int, default=0, help="Tag ID to use")
    parser.add_argument("--config", type=str, default="realsense_D435.yaml")
    parser.add_argument("--resolution", type=str, choices=["low", "medium", "high"], default=None)
    parser.add_argument("--frame", type=str, choices=["opencv", "robot"], default="robot")
    parser.add_argument("--tries", type=int, default=1, help="Number of frames to try")
    args = parser.parse_args()

    ctx = start_realsense(args.config, args.resolution)
    pose = None
    try:
        for _ in range(max(1, args.tries)):
            pose = detect_tag_pose_once(
                ctx,
                tag_family=args.tag_family,
                tag_size_mm=args.tag_size,
                tag_id=args.tag_id,
                frame=args.frame,
            )
            if pose is not None:
                break
    finally:
        stop_realsense(ctx)

    if pose is None:
        print("[NO TAG] Not detected in current frame(s).")
        return

    print(f"[OK] tag_id={pose['tag_id']} frame={pose['frame']} reproj={pose['reproj_error']:.2f}px")
    print(f"position_mm={pose['position_mm']}")
    print(f"rpy_rad={pose['rpy_rad']}")
    print("T_tag_cam=")
    for row in pose["T_tag_cam"]:
        print("  " + " ".join(f"{v: .6f}" for v in row))


if __name__ == "__main__":
    main()
