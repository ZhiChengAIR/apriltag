# Quick Start Guide - Refactored AprilTag System

## Prerequisites

```bash
# Install dependencies
pip install opencv-python numpy pyrealsense2 apriltag pyyaml

# Connect RealSense D435 camera to USB 3.0 port

# Verify camera detected
rs-enumerate-devices
```

## Basic Usage

### 1. Start Tracking

```bash
python3 test_js_1.py
```

### 2. Show AprilTag ID 0 to Camera

The system will automatically detect and track the tag.

### 3. Read Pose Data

**On-screen display:**
- Position (X, Y, Z) in millimeters
- Orientation (Roll, Pitch, Yaw) in radians
- Reprojection error in pixels

**Console output:**
- Updates ~2Hz to reduce overhead
- Format: `[FPS:30.0] X:   0.0 Y: 300.0 Z:   0.0 mm | R:+0.000 P:-0.321 Y:+3.142 rad | err:0.25px`

### 4. Controls

- `q` - Quit
- `s` - Save snapshot
- `c` - Start recording
- `v` - Stop recording

---

## Camera Parameter Configuration

### Default Configuration

The system loads parameters from `realsense_D435.yaml`:

```yaml
realsense:
  rgb_sensor:
    enable_auto_exposure: 0          # CRITICAL: Must be 0
    enable_auto_white_balance: 0     # CRITICAL: Must be 0
    exposure: 8000                   # 8ms exposure time
    gain: 64                         # ISO-like sensitivity
    white_balance: 4500              # Color temperature (Kelvin)
```

### Tuning for Your Environment

**If image is too dark:**
```yaml
exposure: 12000  # Increase exposure (range: 1000-33000)
gain: 80         # Increase gain (range: 0-128)
```

**If image is too bright:**
```yaml
exposure: 5000   # Decrease exposure
gain: 50         # Decrease gain
```

**If colors look wrong:**
```yaml
white_balance: 4000   # Warm lighting (yellow/orange)
white_balance: 5000   # Cool lighting (white/blue)
white_balance: 6500   # Daylight
```

**After editing YAML:**
1. Save file
2. Restart tracker
3. Verify parameters applied (check console output)

---

## Understanding the Output

### Position (X, Y, Z)

Represents **camera position relative to tag** in millimeters:

- **X**: Left (-) / Right (+)
- **Y**: Depth (always positive, distance from tag)
- **Z**: Down (-) / Up (+)

**Example:**
```
X:  50.0 mm  → Camera is 50mm to the right of tag
Y: 300.0 mm  → Camera is 300mm away from tag
Z: 100.0 mm  → Camera is 100mm above tag
```

### Orientation (Roll, Pitch, Yaw)

Represents **camera orientation** in radians:

- **Roll**: Rotation around Y-axis (depth)
- **Pitch**: Rotation around X-axis (left-right)
- **Yaw**: Rotation around Z-axis (up-down)

**Typical values when facing tag:**
```
Roll:  ~0.000 rad  (camera not tilted sideways)
Pitch: ~0.000 rad  (camera level)
Yaw:   ~3.142 rad  (180°, camera facing back toward tag)
```

### Reprojection Error

Indicates pose accuracy in pixels:

- **< 0.5 px**: Excellent
- **0.5 - 1.0 px**: Good
- **> 1.0 px**: Poor (check lighting/tag quality)

---

## Troubleshooting

### Frame Drops

**Symptom:** "FRAME DROP" indicator in bottom-right corner

**Solutions:**
1. Use USB 3.0 port (not USB 2.0)
2. Avoid USB hubs
3. Use shorter/better USB cable
4. Close other applications

**Note:** Frame drops are handled gracefully - UI will not freeze.

---

### No Tag Detected

**Symptom:** "NO TAG DETECTED" message

**Solutions:**
1. Ensure tag is AprilTag ID 0 (from tagStandard41h12 family)
2. Tag must be visible and not occluded
3. Improve lighting (avoid shadows on tag)
4. Move camera closer (optimal range: 0.3m - 1.5m)
5. Check tag is printed clearly (high DPI)
6. Ensure tag is mounted flat (not bent/curved)

---

### High Reprojection Error

**Symptom:** Reprojection error > 1.0 pixels

**Solutions:**
1. **Check auto-exposure is disabled:**
   - Verify YAML has `enable_auto_exposure: 0`
   - Check console output at startup
2. **Improve lighting:**
   - Use even, diffuse lighting (avoid spotlights)
   - Avoid flickering lights (AC-powered LEDs can flicker)
   - Avoid direct sunlight (too bright, causes glare)
3. **Check tag quality:**
   - Print at high DPI (300+ DPI)
   - Use laser printer (not inkjet if possible)
   - Ensure sharp edges
   - Mount on rigid surface (not paper, use foam board)
4. **Verify tag size:**
   - Measure tag with caliper/ruler
   - Update `TAG_SIZE_MM` in code if incorrect

---

### Jittery Pose

**Symptom:** Pose jumps around even when tag is static

**Root Causes & Solutions:**

1. **Auto-exposure enabled (MOST COMMON):**
   ```yaml
   # In YAML, ensure:
   enable_auto_exposure: 0
   ```

2. **Auto-white-balance enabled:**
   ```yaml
   # In YAML, ensure:
   enable_auto_white_balance: 0
   ```

3. **Flickering lights:**
   - Use DC-powered LED lights (no AC flicker)
   - Increase exposure time to average out flicker

4. **Motion blur:**
   - Decrease exposure time
   - Increase lighting to compensate

5. **Tag not rigid:**
   - Mount tag on foam board or acrylic
   - Avoid paper (bends easily)

---

### Camera Not Found

**Symptom:** Error message about RGB sensor not found

**Solutions:**
1. Check USB connection
2. Check camera permissions:
   ```bash
   sudo chmod 666 /dev/bus/usb/*/*
   ```
3. Install/update librealsense:
   ```bash
   # Ubuntu
   sudo apt install librealsense2-utils
   ```
4. Test camera:
   ```bash
   realsense-viewer
   ```

---

## Accuracy Validation

### Quick Visual Check

1. Place tag flat on table
2. Measure distance from tag to camera with ruler (e.g., 300mm)
3. Check Y value in UI
4. Should be within ±5mm of ruler measurement

### Detailed Accuracy Test

```bash
python3 accuracy_test.py
```

**Tests included:**
1. **Distance Test**: Compare detected vs actual distances
2. **Stability Test**: Measure pose variance (jitter)
3. **Translation Test**: Measure detected movement vs actual

**Expected Results:**
- Position accuracy: ±1-3mm
- Orientation accuracy: ±0.005-0.015 rad
- Position std dev: < 1mm
- Orientation std dev: < 0.005 rad

---

## Best Practices

### For Robot Calibration

1. **Mount tag rigidly:**
   - Use foam board or acrylic backing
   - Ensure perfectly flat
   - Fix to stable surface

2. **Optimize lighting:**
   - Use diffuse, non-flickering lights
   - Avoid shadows on tag
   - Consistent lighting across workspace

3. **Configure camera parameters:**
   - Start with YAML defaults
   - Tune exposure/gain for your lighting
   - Disable all auto controls
   - Verify reprojection error < 0.5 px

4. **Collect calibration poses:**
   - 10-20 different poses
   - Vary both position and orientation
   - Cover full robot workspace
   - Include poses at different distances

5. **Validate stability:**
   - Run stability test at representative distance
   - Position std dev should be < 1mm
   - If higher, check lighting and auto-exposure

---

## Performance Targets

### Expected Performance

| Metric | Target | Acceptable |
|--------|--------|------------|
| Frame rate | 30 FPS | 25+ FPS |
| UI update rate | 30+ FPS | 20+ FPS |
| Position accuracy | ±1mm | ±3mm |
| Orientation accuracy | ±0.003 rad | ±0.01 rad |
| Position std dev | < 0.5mm | < 1.0mm |
| Orientation std dev | < 0.001 rad | < 0.005 rad |
| Reprojection error | < 0.5 px | < 1.0 px |

### If Performance Is Poor

**Low frame rate (<20 FPS):**
- USB bandwidth issue (use USB 3.0)
- CPU overloaded (close other apps)

**High jitter (std dev > 2mm):**
- Auto-exposure enabled (check YAML)
- Flickering lights
- Tag not rigid

**High reprojection error (>1 px):**
- Tag print quality poor
- Tag bent/curved
- Incorrect tag size in code
- Lighting too uneven

---

## Common Scenarios

### Scenario 1: Robot Hand-Eye Calibration

**Goal:** Calibrate transform between robot end-effector and camera

**Steps:**
1. Mount tag as world origin (fixed position)
2. Mount camera on robot end-effector
3. Configure camera parameters (disable auto-exposure)
4. Move robot to 15-20 different poses
5. At each pose, record:
   - Robot end-effector pose (from robot controller)
   - Camera pose in tag frame (from this system)
6. Use hand-eye calibration algorithm (e.g., Tsai-Lenz, Horaud)
7. Output: Transform from robot end-effector to camera

**Why this system is ideal:**
- Tag is fixed world reference
- System outputs camera pose in tag frame
- Pose inversion already done (camera-in-tag, not tag-in-camera)
- Fixed camera parameters ensure low jitter

---

### Scenario 2: Robot World Localization

**Goal:** Localize robot in world frame using multiple tags

**Steps:**
1. Mount multiple tags in workspace (known positions)
2. Mount camera on robot
3. As robot moves, detect visible tags
4. For each detected tag:
   - Get camera pose in tag frame (from this system)
   - Transform to world frame (using known tag position)
5. Fuse multiple tag observations
6. Output: Robot pose in world frame

**Integration point:**
```python
# Pseudo-code for integration
T_cam_in_tag = tracker.get_pose()  # This system outputs this
T_tag_in_world = known_tag_poses[tag_id]  # You provide this
T_cam_in_world = T_tag_in_world @ T_cam_in_tag  # Chain transforms
T_robot_in_world = T_cam_in_world @ T_robot_in_cam  # If camera not at robot origin
```

---

## Command Reference

### Main Tracker

```bash
# Start with default config
python3 test_js_1.py

# Start with custom config
# (Edit test_js_1.py line 535 to change config file path)
```

### Accuracy Testing

```bash
# Run full test suite
python3 accuracy_test.py

# Note: accuracy_test.py may need updating with same refactoring
```

### Camera Utilities

```bash
# List RealSense devices
rs-enumerate-devices

# Open RealSense viewer (useful for initial camera setup)
realsense-viewer

# Check camera streams
rs-sensor-control
```

---

## Getting Help

### Check Console Output

The console prints useful information:

```
==================================================================
Camera Intrinsics (Factory Calibrated):
  fx: 617.42 px | fy: 617.65 px
  cx: 321.18 px | cy: 244.52 px
  Distortion: [...]
==================================================================

Applying Camera Parameters:
----------------------------------------------------------------------
  Auto Exposure        = 0
  Auto White Balance   = 0
  Exposure             = 8000
  Gain                 = 64
  White Balance        = 4500
----------------------------------------------------------------------
```

If you don't see "Applying Camera Parameters", the YAML may not be loading.

### Enable Debug Output

Add to top of `test_js_1.py` `run()` method:

```python
print(f"Config loaded: {self.camera_params}")
print(f"RGB sensor found: {self._find_rgb_sensor() is not None}")
```

---

## Summary Checklist

Before running robot calibration:

- [ ] RealSense D435 connected to USB 3.0 port
- [ ] AprilTag ID 0 printed clearly at correct size
- [ ] Tag mounted rigidly on flat surface
- [ ] `realsense_D435.yaml` configured for your lighting
- [ ] `enable_auto_exposure: 0` in YAML
- [ ] `enable_auto_white_balance: 0` in YAML
- [ ] Even, non-flickering lighting on tag
- [ ] Tracker runs at 30 FPS without frequent frame drops
- [ ] Reprojection error < 1.0 pixels
- [ ] Stability test shows position std dev < 1mm

If all checkmarks satisfied → System is ready for calibration!
