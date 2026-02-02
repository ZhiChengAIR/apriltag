# Coordinate System Explanation

## Left-Handed Coordinate System (YOUR SYSTEM)

```
         Z-axis (Up/Down)
         ↑
         |
         |
         |
         +------→ X-axis (Left/Right)
        /
       /
      ↙
   Y-axis (In/Out, Depth)
```

### Axes Definition:
- **X-axis**: Left/Right (positive = right when looking at tag)
- **Y-axis**: In/Out, Depth (positive = away from tag, into the scene)
- **Z-axis**: Up/Down (positive = upward)

---

## What the Code Calculates

The code calculates **CAMERA POSE RELATIVE TO TAG** in the left-handed coordinate system.

### Scenario:
- Tag is **FIXED** in space (tag is the reference frame origin)
- Camera is **MOVING** relative to the tag
- We measure where the camera is positioned and how it's oriented

---

## Example Readings

### Example 1: Camera Directly in Front of Tag at 300mm
```
Position (mm):
  X:    0.00 mm   (Camera centered horizontally)
  Y:  300.00 mm   (Camera is 300mm away from tag)
  Z:    0.00 mm   (Camera at same height as tag)

Orientation (rad):
  Roll:   0.0000 rad  (No rotation around Y-axis)
  Pitch:  0.0000 rad  (No rotation around X-axis)
  Yaw:    3.1416 rad  (180°, camera facing back toward tag)
```

### Example 2: Camera 50mm to the Right, 300mm Away
```
Position (mm):
  X:   50.00 mm   (Camera shifted 50mm to the right)
  Y:  300.00 mm   (Still 300mm away)
  Z:    0.00 mm   (Same height)

Orientation (rad):
  Roll:   0.0000 rad
  Pitch:  0.0000 rad
  Yaw:    3.1416 rad  (Still facing tag)
```

### Example 3: Camera 100mm Above Tag, 300mm Away
```
Position (mm):
  X:    0.00 mm   (Centered horizontally)
  Y:  300.00 mm   (300mm away)
  Z:  100.00 mm   (100mm above tag)

Orientation (rad):
  Roll:   0.0000 rad
  Pitch: -0.3217 rad  (Tilted down ~18.4° to look at tag)
  Yaw:    3.1416 rad
```

---

## Understanding the 3D Visualization in UI

The UI shows a **3D coordinate visualization** in the top-right corner.

### What You See:
- **Gray Square**: The AprilTag (reference frame origin)
- **Thin Axes from Tag**: Tag's coordinate system (reference)
  - Horizontal Red: Tag's X-axis (left/right)
  - Vertical Blue: Tag's Z-axis (up/down)

- **Thick Colored Arrows**: Camera's coordinate axes
  - **Red Arrow**: Camera's X-axis direction
  - **Green Arrow**: Camera's Y-axis direction (depth)
  - **Blue Arrow**: Camera's Z-axis direction

### How to Interpret:
- **Arrow directions** show how the camera is oriented
- **Arrow lengths** are for visualization only (not to scale)
- **If green arrow points away from you**: Camera Y-axis (depth) points outward
- **If red arrow points right**: Camera is not rolled
- **If blue arrow points up**: Camera is not pitched

---

## Coordinate Transformation Details

### What solvePnP Returns:
OpenCV's `solvePnP` returns the **tag pose in camera frame** using right-handed coordinates:
- X: right
- Y: down
- Z: forward (out from camera)

### What We Convert To:

#### Step 1: Invert the Transformation
Convert from "tag in camera frame" → "camera in tag frame"

```python
R_camera_in_tag = R_tag_in_camera^T
t_camera_in_tag = -R_camera_in_tag * t_tag_in_camera
```

#### Step 2: Transform to Left-Handed System
Apply coordinate transformation:

```
X_new = X_old      (left/right stays same)
Y_new = Z_old      (depth becomes Y)
Z_new = -Y_old     (up becomes Z, flipped)
```

This gives you the **camera position and orientation in tag frame using left-handed coordinates**.

---

## Units

### All Outputs are in MILLIMETERS (mm):
- Position X, Y, Z: **millimeters (mm)**
- NOT centimeters, NOT meters
- Example: Y = 300.0 mm means 30 cm = 0.3 m

### All Rotations are in RADIANS (rad):
- Roll, Pitch, Yaw: **radians (rad)**
- NOT degrees
- Conversion: 1 rad = 57.3°
- Example: Yaw = 3.1416 rad ≈ 180°

---

## Testing Accuracy

### Quick Test:
1. Place tag flat on table
2. Hold camera **directly above tag** at known distance (e.g., 300mm measured with ruler)
3. Check reading:
   - X should be ~0 (centered)
   - Y should be close to measured distance
   - Z should be ~0 (same plane if camera center is at tag height)

### Why Y is Depth (Not Z):
In **left-handed coordinate systems** commonly used in robotics and graphics:
- Z is vertical (gravity direction)
- Y is forward/backward (depth into scene)
- X is left/right (horizontal)

This matches **Unity, Direct3D, and many robotics conventions**.

---

## Troubleshooting

### Q: Why is Y larger/smaller than my ruler measurement?
**A**: Several reasons:
1. Y measures from tag center to camera optical center (not camera housing)
2. Camera optical center is ~5-10mm inside the camera body
3. AprilTag accuracy is ±1-3mm at 300mm distance

### Q: Why do I see small jitter in readings (±0.5mm)?
**A**: Normal! Causes:
1. Sub-pixel corner detection uncertainty
2. Image noise
3. Motion blur from breathing/hand movement
4. Run `accuracy_test.py` stability test to measure jitter

### Q: My X/Z readings are not exactly zero when centered
**A**: Normal! Causes:
1. Tag not perfectly aligned with camera
2. Tag not perfectly flat
3. Small measurement errors (~1-2mm is normal)

### Q: Readings jump by large amounts (>10mm)
**A**: Problem! Check:
1. Lighting (shadows on tag?)
2. Tag print quality (sharp edges?)
3. Tag flatness (bent/curved?)
4. Reprojection error in UI (should be <1.0 px)

---

## Summary

✅ **Units**: All distances in mm, all angles in rad
✅ **Coordinate System**: Left-handed (X=L/R, Y=Depth, Z=U/D)
✅ **Reference Frame**: Tag is origin, camera pose is measured relative to tag
✅ **Accuracy**: Expect ±1-3mm position, ±0.005-0.015 rad rotation
✅ **Visualization**: 3D viz in UI shows camera orientation

For detailed accuracy validation, run:
```bash
python accuracy_test.py
```
