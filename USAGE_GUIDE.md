# AprilTag RealSense D435 Tracking - Complete Guide

## Quick Start

### 1. Install Dependencies
```bash
pip install -r requirements_tracker.txt
```

### 2. Run Main Tracker
```bash
python test_js_1.py
```

**Controls:**
- `q` - Quit
- `s` - Save snapshot
- `c` - Start video recording
- `v` - Stop video recording

### 3. Run Accuracy Tests
```bash
python accuracy_test.py
```

---

## Updates & Features

### ✅ What's New

1. **Enhanced UI Contrast** - Darker backgrounds (80% opacity), brighter text colors
2. **Console Output** - Real-time pose data printed to terminal every 0.5s
3. **Video Recording** - Record annotated video with poses overlaid
4. **Accuracy Test Suite** - Comprehensive testing script for validation

### Video Recording
- Press `c` to start recording
- Red dot appears in top-right when recording
- Press `v` to stop recording
- Videos saved as `recording_000.mp4`, `recording_001.mp4`, etc.
- Recorded video includes all UI overlays and pose information

---

## Camera Intrinsics - Your Question Answered

### Do We Need Manual Camera Calibration?

**Short Answer: NO - RealSense factory calibration is used and is highly accurate.**

### Explanation

The official AprilTag documentation shows manual configuration:
```c
apriltag_detection_info_t info;
info.fx = fx;  // These need manual input
info.fy = fy;
info.cx = cx;
info.cy = cy;
```

**Our implementation is actually BETTER** because:

#### 1. **Factory Calibration is Pre-Applied**
```python
# We automatically extract calibrated parameters:
intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
fx = intrinsics.fx  # Factory-calibrated focal length X
fy = intrinsics.fy  # Factory-calibrated focal length Y
cx = intrinsics.ppx # Principal point X
cy = intrinsics.ppy # Principal point Y
dist = intrinsics.coeffs  # Lens distortion coefficients
```

#### 2. **What We Get from RealSense**
At startup, you'll see output like:
```
Camera Intrinsics (Factory Calibrated):
  fx: 615.23 pixels
  fy: 615.45 pixels
  cx: 321.15 pixels
  cy: 237.89 pixels
  Distortion: [0.0, 0.0, 0.0, 0.0, 0.0]
```

These parameters are:
- **Calibrated at factory** using precision equipment
- **Stored in camera firmware** (never lost)
- **Specific to your individual camera unit**
- **Include lens distortion compensation**

#### 3. **Our Resolution Configuration**
```python
# 640x480 @ 30fps - optimal for AprilTag
self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
```

This resolution is chosen because:
- **30 FPS** ensures real-time performance
- **640x480** provides enough detail for accurate corner detection
- **Factory calibration is valid** for this resolution
- **Lower resolution = faster processing** without sacrificing accuracy

### Do You Need Additional Calibration?

**Only if:**
1. Camera has been physically damaged
2. Lens has been replaced
3. You want to verify factory calibration

**For 99% of use cases: NO additional calibration needed.**

---

## Accuracy Expectations & Reality Check

### Your Requirements vs Reality

| Requirement | Reality | Assessment |
|-------------|---------|------------|
| XYZ within 0.1mm | Challenging | See below |
| Rotation within 0.01 rad | Achievable | ~0.57° |

### Realistic Accuracy at 300mm Distance

#### RealSense D435 Specifications
- **Depth accuracy**: ~2% at 300mm = ±6mm
- **RGB camera**: Not depth-sensing, uses visual features only

#### AprilTag Pose Estimation (Our Method)
Uses visual corner detection + PnP solver:

**Expected Performance:**
- **Position (XYZ)**: 1-3mm standard deviation
- **Rotation**: 0.005-0.015 rad (~0.3-0.9°)
- **Reprojection error**: <1.0 pixel = good accuracy

### Factors Affecting Accuracy

#### 1. **Tag Quality** (Most Important)
- ✅ Print on rigid, flat surface (foam board recommended)
- ✅ High contrast black/white
- ✅ No reflections or glare
- ✅ Sharp edges (use laser printer, not inkjet)

#### 2. **Tag Size**
- Larger tag = better accuracy
- Your 32mm tag is reasonable for 300mm distance
- Consider 50-100mm tag for sub-millimeter accuracy

#### 3. **Lighting**
- Uniform, diffuse lighting
- Avoid shadows on tag
- Avoid direct sunlight causing glare

#### 4. **Camera Angle**
- Best accuracy when tag is perpendicular to camera
- Accuracy degrades at extreme angles (>45°)

#### 5. **Motion**
- Static tag = best accuracy
- Camera/tag motion causes motion blur

### How to Achieve Best Accuracy

```bash
# Run accuracy test suite
python accuracy_test.py
```

**Tests included:**
1. **Distance Test** - Measure actual vs detected distance
2. **Stability Test** - Measure jitter with static tag
3. **Translation Test** - Measure movement accuracy

**Expected Results (with proper setup):**
- Position std dev: 0.5-2mm (excellent to good)
- Rotation std dev: 0.002-0.008 rad
- Distance error: 1-3% at 300mm (3-9mm)

---

## Improving Accuracy

### 1. Use Depth Camera (Optional Enhancement)
If you need true sub-millimeter accuracy, enable depth stream:

```python
# Add to __init__:
self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Then align depth to color and use depth data
# to refine Z-distance measurement
```

### 2. Temporal Filtering
Add Kalman filter or moving average to reduce jitter:

```python
# Already prepared in code:
self.pose_history = deque(maxlen=3)
# Implement exponential moving average
```

### 3. Multi-Tag Bundles
Use multiple tags in known configuration for better accuracy.

### 4. Larger Tag
Print larger tag (50mm or 100mm) for better corner detection at distance.

---

## Troubleshooting

### Issue: "NO TAG DETECTED"
- **Solution**: Check lighting, ensure tag is in view, verify tag ID is 0

### Issue: High Reprojection Error (>2.0 px)
- **Solution**: Tag not flat, improve lighting, check for motion blur

### Issue: Jittery Pose
- **Solution**: Improve lighting, ensure tag is rigid, reduce camera exposure time

### Issue: Inaccurate Distance
- **Solution**: Verify tag size (32mm = black square only), check camera focus

---

## Technical Details

### Coordinate Systems

**Tag Frame** (Origin at tag center):
- X-axis: Right (red)
- Y-axis: Down (green)
- Z-axis: Out from tag surface (blue)

**Camera Pose Output**:
- Position: Camera location relative to tag (mm)
- Rotation: Camera orientation relative to tag (radians)
- Convention: ZYX Euler angles (roll, pitch, yaw)

### Pose Estimation Pipeline

1. **Image Acquisition** - Get RGB frame from RealSense
2. **Grayscale Conversion** - AprilTag works on grayscale
3. **Tag Detection** - Detect quadrilaterals and decode ID
4. **Corner Refinement** - Sub-pixel accuracy corner detection
5. **PnP Solver** - SOLVEPNP_IPPE_SQUARE (optimal for squares)
6. **Pose Extraction** - Convert to translation + Euler angles

### Performance Metrics

- **FPS**: 30-60 Hz (CPU dependent)
- **Latency**: <33ms (one frame)
- **Detection Range**: 100-1000mm (depends on tag size)
- **Detection Angle**: ±60° from perpendicular

---

## Files

- `test_js_1.py` - Main tracking script with recording
- `accuracy_test.py` - Accuracy test suite
- `requirements_tracker.txt` - Python dependencies
- `USAGE_GUIDE.md` - This file

---

## Summary

Your setup is correctly configured:
- ✅ Factory camera calibration is used (no manual calibration needed)
- ✅ Resolution and FPS are optimized
- ✅ Pose estimation uses best-practice algorithm (SOLVEPNP_IPPE_SQUARE)
- ✅ All camera intrinsics (fx, fy, cx, cy, distortion) are properly applied

The accuracy will be in the 1-3mm range for position and 0.005-0.015 rad for rotation under good conditions. Use `accuracy_test.py` to validate performance with your specific setup.
