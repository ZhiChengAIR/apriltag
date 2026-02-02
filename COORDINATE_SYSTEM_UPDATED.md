# AprilTag Coordinate System Reference

## Overview

This implementation uses the **exact coordinate system** as defined by the AprilTag library authors. No coordinate transformations are applied.

---

## Camera Coordinate Frame

**Origin:** Camera optical center

**Axes:**
- **X-axis:** Right in the image (positive = right, negative = left)
- **Y-axis:** Down in the image (positive = down, negative = up)
- **Z-axis:** Forward out of camera lens (positive = depth/forward)

```
        Y (down)
        |
        |
        +------ X (right)
       /
      /
     Z (forward/depth)
```

---

## Tag Coordinate Frame

**Origin:** Center of the tag

**Axes (from viewer perspective looking at tag):**
- **X-axis:** Right (positive = right, negative = left)
- **Y-axis:** Down (positive = down, negative = up)
- **Z-axis:** Into the tag plane (positive = into tag, negative = out toward camera)

```
Tag viewed from camera:

    +-------+
    |       |
    |   O   |  O = tag center (origin)
    |       |
    +-------+

    X = right
    Y = down
    Z = into tag (away from camera)
```

---

## Pose Output

The system outputs: **Tag pose in Camera frame**

**Translation (X, Y, Z) in millimeters:**
- **X:** How far right the tag is from camera center
- **Y:** How far down the tag is from camera center
- **Z:** How far forward/deep the tag is from camera (distance)

**Rotation (roll, pitch, yaw) in radians:**
- **Roll:** Rotation around X-axis (image left-right)
- **Pitch:** Rotation around Y-axis (image up-down)
- **Yaw:** Rotation around Z-axis (image rotation)

---

## Example Scenarios

### Scenario 1: Tag Directly in Front, 300mm Away

```
Position (mm):
  X:    0.0  (centered horizontally)
  Y:    0.0  (centered vertically)
  Z:  300.0  (300mm forward/depth)

Orientation (rad):
  Roll:   0.0  (no rotation)
  Pitch:  0.0  (no rotation)
  Yaw:    0.0  (no rotation)
```

### Scenario 2: Tag 50mm to the Right, 300mm Away

```
Position (mm):
  X:   50.0  (50mm to the right)
  Y:    0.0  (centered vertically)
  Z:  300.0  (300mm forward)
```

### Scenario 3: Tag 100mm Below Center, 300mm Away

```
Position (mm):
  X:    0.0  (centered horizontally)
  Y:  100.0  (100mm down)
  Z:  300.0  (300mm forward)
```

---

## Important Notes

### Z-Axis is Depth

The **Z value is the distance** from camera to tag. This should match what you measure with a ruler.

If Z = 300mm, the tag should be approximately 300mm away from the camera.

### Y-Axis is Down

**Positive Y = DOWN in image**, not up. This matches:
- OpenCV convention
- AprilTag library convention
- Most computer vision conventions

### No Coordinate Transformations

The output is **directly from pose estimation** with no left-handed/right-handed conversions or inversions. What you see is what the AprilTag library calculates.

---

## Verifying Accuracy

### Quick Test

1. Place tag flat on table
2. Measure distance from camera to tag with ruler (e.g., 300mm)
3. Check Z value in display
4. Z should be approximately equal to measured distance (±5mm)

### Check Reprojection Error

Press **'d'** for debug output. Look at reprojection error:

- **< 1 pixel:** Excellent
- **1-2 pixels:** Good
- **> 10 pixels:** Wrong tag size or bad setup

### If Z Distance is Wrong

**Most common cause: Wrong tag size**

1. Run the measurement tool:
   ```bash
   python3 measure_tag_size.py
   ```

2. It will test different tag sizes and tell you the correct one

3. Update `TAG_SIZE_MM` in `test_js_1.py` (line ~590)

---

## Controls Reference

| Key | Action | Output |
|-----|--------|--------|
| **o** | Toggle display overlay | - |
| **c** | Start recording | `data/recording_XXX.mp4` |
| **s** | Stop recording | - |
| **x** | Take snapshot | `data/snapshot_XXX.png` |
| **d** | Print debug info | Console output |
| **q** | Quit | - |

All files are saved to the `data/` folder.

---

## Technical Details

### Pose Estimation

Uses OpenCV's `solvePnP` with `SOLVEPNP_IPPE_SQUARE` method, which:
- Assumes planar target (tag is flat)
- Uses iterative refinement
- Returns tag pose in camera frame
- Follows standard computer vision conventions

### Corner Ordering

Corners are ordered as:
0. Top-left
1. Top-right
2. Bottom-right
3. Bottom-left

(Clockwise from top-left)

### Object Points

3D points of tag corners in tag frame (tag lying flat in XY plane):
```python
half = tag_size / 2
object_points = [
    [-half, -half, 0],  # top-left
    [ half, -half, 0],  # top-right
    [ half,  half, 0],  # bottom-right
    [-half,  half, 0]   # bottom-left
]
```

### Rotation Representation

Internally uses:
- Rotation vector (rvec) from solvePnP
- Converted to rotation matrix for display
- Converted to Euler angles (roll, pitch, yaw) for UI

Euler angle convention: ZYX (yaw-pitch-roll)

---

## Troubleshooting

### Z Distance is Wrong

1. **Verify tag size:**
   ```bash
   python3 measure_tag_size.py
   ```

2. **Measure actual tag size** with caliper/ruler

3. **Update TAG_SIZE_MM** if needed

### High Reprojection Error

1. **Tag size mismatch** (most common)
2. **Tag not flat** (bent/curved)
3. **Poor lighting** (shadows on tag)
4. **Low print quality** (blurry edges)

### Position Jumps Around

1. **Auto-exposure enabled** - Check YAML has `enable_auto_exposure: 0`
2. **Flickering lights** - Use DC-powered LEDs
3. **Tag not rigid** - Mount on foam board

---

## Comparison with Other Conventions

| System | X | Y | Z |
|--------|---|---|---|
| **AprilTag/OpenCV** | Right | Down | Forward |
| Unity/Direct3D (Left-handed) | Right | Up | Forward |
| ROS (REP-103) | Forward | Left | Up |
| OpenGL (Right-handed) | Right | Up | Backward |

**This implementation uses AprilTag/OpenCV convention** as specified by the library authors.

---

## References

- AprilTag Library: https://github.com/AprilRobotics/apriltag
- OpenCV solvePnP: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
- Coordinate System Documentation: https://april.eecs.umich.edu/software/apriltag
