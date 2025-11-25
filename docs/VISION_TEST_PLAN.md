# PhotonVision Integration Test Plan

## Overview
This document provides a comprehensive test plan for validating the PhotonVision integration and AprilTag navigation capabilities of MainBot2025.

---

## 1. PhotonVision Integration Review

### 1.1 Vision Subsystem Architecture
- **Camera**: ArduCam1 (configured in PhotonVision dashboard)
- **Pose Estimation Strategy**: `MULTI_TAG_PNP_ON_COPROCESSOR` (best accuracy)
- **Fallback Strategy**: `LOWEST_AMBIGUITY` (for single-tag scenarios)
- **Field Layout**: 2024 Crescendo field layout
- **Dashboard URL**: http://192.168.86.30:5800/#/camera

### 1.2 Key Features
- ✅ Real-time AprilTag detection and pose estimation
- ✅ Multi-tag pose estimation for enhanced accuracy
- ✅ Integration with swerve drivetrain pose estimation (vision fusion)
- ✅ Health monitoring and diagnostics
- ✅ Telemetry and logging

### 1.3 Vision Fusion
The vision subsystem is integrated with the drivetrain's pose estimator:
- Vision measurements are continuously fused with odometry in `Robot.java` (`updateVisionFusion()`)
- Standard deviations are adjusted based on number of tags detected (more tags = higher confidence)
- Vision measurements improve overall pose accuracy during autonomous navigation

### 1.4 Pose Reset Test Utility
A test utility is available for resetting the robot's pose on the field during testing:
- **Dashboard Widget**: "TEST: Reset Robot Pose" chooser and "TEST: Reset Pose to Selected" button
- **Purpose**: Allows quick repositioning of robot for consistent test starting positions
- **Available Positions**: Home, Field Center, Origin, Blue/Red start positions
- **Location**: Shuffleboard/Glass dashboard under "TEST: Reset Robot Pose"
- **Note**: This is a temporary testing utility and can be removed after testing (see `POSE_RESET_TEST_UTILITY_README.md`)

---

## 2. Available Commands for Testing

### 2.1 Command Overview

| Command | Purpose | Distance Control | Vision Usage |
|---------|---------|------------------|--------------|
| `DriveToAprilTag2Command` | Simple PID-based approach to Tag 2 | ❌ Drives to tag center (0.5m tolerance) | ✅ Uses vision for pose estimation |
| `VisionAssistedAprilTagCommand` | Two-phase navigation (PathPlanner + Vision) | ✅ 0.5m from tag (hardcoded) | ✅ Vision precision control |
| `DriveToAprilTagOffsetCommand` | PathPlanner-based with configurable distance | ✅ Configurable distance | ❌ PathPlanner only |
| `DriveToAprilTagCommand` | PathPlanner to tag center | ❌ Tag center | ❌ PathPlanner only |
| `DriveToAprilTagOffsetOrVisionCommand` | Offset with vision fallback | ✅ Configurable distance | ✅ Uses vision if available |

### 2.2 Current Button Mappings

| Button Combination | Command | Tag ID | Notes |
|-------------------|---------|--------|-------|
| **X + Y** | `DriveToAprilTag2Command` | 2 | Simple PID approach |
| **X + B** | `VisionAssistedAprilTagCommand` | 1 | Two-phase navigation |
| **Y + B** | `VisionAssistedAprilTagCommand` | 3 | Two-phase navigation |
| **X + A** | `VisionAssistedAprilTagCommand` | 4 | Two-phase navigation |
| **Start** | `DriveToAprilTag2Command` | 2 | Vision test |
| **D-pad Down** | `DriveToAprilTag2Command` | 2 | Vision test |

---

## 3. Test Plan for Tag 2 and Tag 3 Navigation

### 3.1 Pre-Test Checklist

#### Vision System Verification
- [ ] PhotonVision dashboard accessible (http://192.168.86.30:5800/#/camera)
- [ ] Camera "ArduCam1" is connected and streaming
- [ ] AprilTags 2 and 3 are visible in camera feed
- [ ] Vision subsystem shows "Connected: true" in SmartDashboard
- [ ] At least one AprilTag is detected (check "Vision/Has Targets")

#### Robot Setup
- [ ] Robot is powered on and drivetrain is functional
- [ ] Robot is positioned on field with clear line of sight to test tags
- [ ] Field layout is correct (2024 Crescendo)
- [ ] Robot pose is initialized correctly (check Field2d widget)
- [ ] Pose reset utility is available on dashboard (check for "TEST: Reset Robot Pose" widget)

#### Safety
- [ ] Emergency stop accessible
- [ ] Clear test area (no obstacles between robot and tags)
- [ ] Robot has sufficient battery charge
- [ ] Test area is clear of people and obstacles

---

### 3.2 Test Scenarios

#### Test 1: Basic Vision Detection (Tag 2)
**Objective**: Verify camera can detect AprilTag 2

**Steps**:
1. Position robot with Tag 2 in camera view
2. Check SmartDashboard:
   - `Vision/Connected` = true
   - `Vision/Has Targets` = true
   - `Vision/Best Target ID` = 2
   - `Vision/Best Target Ambiguity` < 0.3
3. Verify PhotonVision dashboard shows Tag 2 detection

**Expected Results**:
- Tag 2 is consistently detected
- Ambiguity is low (< 0.3)
- Pose estimation is updating

**Pass Criteria**: Tag 2 detected with ambiguity < 0.3

---

#### Test 2: Basic Vision Detection (Tag 3)
**Objective**: Verify camera can detect AprilTag 3

**Steps**:
1. Position robot with Tag 3 in camera view
2. Check SmartDashboard for Tag 3 detection
3. Verify PhotonVision dashboard shows Tag 3 detection

**Expected Results**: Same as Test 1, but for Tag 3

**Pass Criteria**: Tag 3 detected with ambiguity < 0.3

---

#### Test 3: Drive to Tag 2 (Simple PID - Button Test)
**Objective**: Test simple PID-based navigation to Tag 2

**Steps**:
1. **Optional**: Use pose reset utility to set robot to a known starting position (e.g., "Home Position")
2. Position robot 2-3 meters away from Tag 2 (or reset pose to a position 2-3m from Tag 2)
3. Press **X + Y** (or **Start** button)
4. Observe robot movement
5. Monitor SmartDashboard telemetry

**Expected Results**:
- Robot moves toward Tag 2
- Console shows position updates
- Robot stops within 0.5m of Tag 2 center

**Pass Criteria**: Robot reaches Tag 2 within 0.5m tolerance

**Issues to Watch For**:
- Robot overshoots target
- Robot oscillates around target
- Robot doesn't reach target (check PID gains)

---

#### Test 4: Drive to Tag 3 (Vision-Assisted - Button Test)
**Objective**: Test two-phase navigation to Tag 3

**Steps**:
1. **Optional**: Use pose reset utility to set robot to a known starting position
2. Position robot 3-4 meters away from Tag 3 (or reset pose to a position 3-4m from Tag 3)
3. Press **Y + B**
4. Observe two-phase behavior:
   - Phase 1: PathPlanner coarse navigation
   - Phase 2: Vision precision control
4. Monitor SmartDashboard:
   - `VisionAssisted/Phase` should show "PATHPLANNER" then "VISION"
   - `VisionAssisted/Distance to Target` should decrease

**Expected Results**:
- Robot uses PathPlanner to get within 1.5m
- Switches to vision control automatically
- Final position: 0.5m from Tag 3, facing the tag

**Pass Criteria**: Robot reaches 0.5m from Tag 3, facing tag

**Issues to Watch For**:
- Phase transition doesn't occur
- Vision phase doesn't activate
- Final positioning is inaccurate

---

#### Test 5: Drive to Tag 2 with Custom Distance (Autonomous)
**Objective**: Test autonomous navigation to Tag 2 at specific distance

**Steps**:
1. **Recommended**: Use pose reset utility to set robot to a consistent starting position (e.g., "Home Position" or "Blue Start Center")
2. Select "Vision Test: Tag 2 (1.0m)" from auto chooser
3. Enable autonomous mode
4. Observe robot navigation
5. Measure final distance from tag
6. **For repeatability**: Reset pose to same starting position and repeat test

**Expected Results**:
- Robot navigates to Tag 2
- Final position is 1.0m from tag center
- Robot faces the tag

**Pass Criteria**: Final distance is 1.0m ± 0.1m

---

#### Test 6: Drive to Tag 3 with Custom Distance (Autonomous)
**Objective**: Test autonomous navigation to Tag 3 at specific distance

**Steps**:
1. **Recommended**: Use pose reset utility to set robot to a consistent starting position
2. Select "Vision Test: Tag 3 (1.5m)" from auto chooser
3. Enable autonomous mode
4. Observe robot navigation
5. Measure final distance from tag
6. **For repeatability**: Reset pose to same starting position and repeat test

**Expected Results**:
- Robot navigates to Tag 3
- Final position is 1.5m from tag center
- Robot faces the tag

**Pass Criteria**: Final distance is 1.5m ± 0.1m

---

#### Test 7: Multiple Distance Tests (Tag 2)
**Objective**: Verify accuracy at different distances

**Test Distances**: 0.5m, 1.0m, 1.5m, 2.0m

**Steps**:
1. **Recommended**: Use pose reset utility to set robot to a consistent starting position for all tests
2. For each distance:
   - Select corresponding auto from chooser (e.g., "Vision Test: Tag 2 (0.5m)")
   - Run autonomous mode
   - Measure actual distance achieved
   - Record results
   - Reset pose to starting position before next test

**Expected Results**: Robot consistently reaches target distance ± 0.1m

**Pass Criteria**: All distances within ± 0.1m tolerance

---

#### Test 8: Multiple Distance Tests (Tag 3)
**Objective**: Verify accuracy at different distances for Tag 3

**Test Distances**: 0.5m, 1.0m, 1.5m, 2.0m

**Steps**: 
1. **Recommended**: Use pose reset utility to set robot to a consistent starting position for all tests
2. For each distance:
   - Select corresponding auto from chooser (e.g., "Vision Test: Tag 3 (0.5m)")
   - Run autonomous mode
   - Measure actual distance achieved
   - Record results
   - Reset pose to starting position before next test

**Pass Criteria**: All distances within ± 0.1m tolerance

---

#### Test 9: Vision Fusion Accuracy
**Objective**: Verify vision measurements improve pose estimation

**Steps**:
1. Position robot with Tag 2 visible
2. Disable vision (set `ENABLE_POSE_ESTIMATION = false`)
3. Record odometry-only pose
4. Enable vision
5. Record vision-fused pose
6. Compare accuracy (use known tag position as ground truth)

**Expected Results**:
- Vision-fused pose is more accurate than odometry-only
- Pose updates when vision detects tags
- Standard deviations decrease with more tags

**Pass Criteria**: Vision fusion improves pose accuracy by > 10%

---

#### Test 10: Edge Cases
**Objective**: Test behavior in challenging scenarios

**Scenarios**:
1. **Tag partially occluded**: Position robot so tag is partially blocked
2. **Multiple tags visible**: Position robot to see both Tag 2 and Tag 3
3. **Tag at maximum distance**: Test at ~5m (MAX_APRILTAG_DISTANCE_METERS)
4. **Tag at angle**: Test with robot approaching tag from side (not front)
5. **Vision loss during navigation**: Block camera view mid-navigation

**Expected Results**:
- Robot handles partial occlusion gracefully
- Multi-tag detection improves accuracy
- Robot stops safely if vision is lost
- Navigation continues using odometry if vision unavailable

**Pass Criteria**: Robot handles all edge cases without crashing or unsafe behavior

---

## 4. Telemetry and Monitoring

### 4.1 Key SmartDashboard Values to Monitor

#### Vision Subsystem
- `Vision/Connected` - Camera connection status
- `Vision/Has Targets` - Whether any tags are detected
- `Vision/Target Count` - Number of tags visible
- `Vision/Best Target ID` - ID of best detected tag
- `Vision/Best Target Ambiguity` - Lower is better (< 0.3 is good)
- `Vision/Estimated Pose` - Current vision-based pose estimate
- `Vision/Tags Used` - Number of tags used for pose estimation

#### Vision-Assisted Commands
- `VisionAssisted/Phase` - Current phase (PATHPLANNER or VISION)
- `VisionAssisted/Distance to Target` - Distance to target position
- `VisionAssisted/Good Vision` - Whether good vision target is available
- `VisionAssisted/Elapsed Time` - Total command execution time
- `VisionAssisted/Status` - Current status message

#### Field Widget
- Robot pose (blue robot icon)
- AprilTag positions (red squares)
- Target position (if applicable)

#### Pose Reset Test Utility
- `TEST: Reset Robot Pose` - Dropdown chooser with predefined positions
- `TEST: Reset Pose to Selected` - Button to trigger pose reset
- Available positions: Home, Field Center, Origin, Blue/Red start positions

### 4.2 Console Logging
Monitor console for:
- `[VisionSubsystem]` messages - Vision system status
- `[VisionAssisted]` messages - Command phase transitions
- `DriveToAprilTag2:` messages - Position updates
- `[PoseResetTest]` messages - Pose reset confirmation messages
- Error messages - Any exceptions or failures

### 4.3 Using the Pose Reset Utility

**How to Reset Robot Pose:**
1. Open Shuffleboard or Glass dashboard
2. Locate the "TEST: Reset Robot Pose" dropdown chooser
3. Select desired position from the list:
   - **Home Position (3.0, 3.0, 0°)** - Default safe position
   - **Field Center** - Center of the field
   - **Origin (0, 0, 0°)** - Field origin
   - **Blue Start Left/Center/Right** - Blue alliance starting positions
   - **Red Start Left/Center/Right** - Red alliance starting positions
4. Click the "TEST: Reset Pose to Selected" button
5. Verify robot pose updated in Field2d widget
6. Console will show: `[PoseResetTest] Robot pose reset to: (x, y, angle)`

**Best Practices:**
- Reset pose before each test for consistent starting conditions
- Use the same starting position for repeated tests to ensure repeatability
- Verify pose reset in Field2d widget before running autonomous commands
- Note the starting position in test results for reference

---

## 5. Troubleshooting Guide

### 5.1 Common Issues

#### Issue: Camera not connected
**Symptoms**: `Vision/Connected` = false
**Solutions**:
- Check camera power and network connection
- Verify camera name matches `PRIMARY_CAMERA_NAME` ("ArduCam1")
- Check PhotonVision dashboard accessibility
- Restart PhotonVision service

#### Issue: No tags detected
**Symptoms**: `Vision/Has Targets` = false
**Solutions**:
- Verify tags are in camera field of view
- Check lighting conditions (tags need good lighting)
- Verify tag IDs exist in field layout
- Check camera focus and exposure settings in PhotonVision dashboard

#### Issue: High ambiguity
**Symptoms**: `Vision/Best Target Ambiguity` > 0.3
**Solutions**:
- Move robot closer to tag
- Ensure tag is fully visible (not partially occluded)
- Check camera angle (should be relatively perpendicular to tag)
- Verify camera calibration in PhotonVision dashboard

#### Issue: Robot doesn't reach target
**Symptoms**: Robot stops before reaching tag
**Solutions**:
- Check PID controller gains (may need tuning)
- Verify tolerance settings aren't too large
- Check for obstacles blocking path
- Verify target pose calculation is correct

#### Issue: Robot overshoots target
**Symptoms**: Robot passes tag and oscillates
**Solutions**:
- Reduce PID P gain
- Add D (derivative) term to PID controllers
- Reduce maximum speed during precision phase
- Check for vision latency causing delayed updates

#### Issue: Phase transition doesn't occur
**Symptoms**: Stays in PATHPLANNER phase
**Solutions**:
- Check `VISION_SWITCH_DISTANCE` threshold (1.5m)
- Verify vision target is available (`VisionAssisted/Good Vision`)
- Check distance calculation (`VisionAssisted/Distance to Target`)
- Verify PathPlanner command completes

---

## 6. Test Results Template

### Test Execution Log

| Test # | Test Name | Date | Result | Notes |
|--------|-----------|------|--------|-------|
| 1 | Basic Vision Detection (Tag 2) | | PASS/FAIL | |
| 2 | Basic Vision Detection (Tag 3) | | PASS/FAIL | |
| 3 | Drive to Tag 2 (Button) | | PASS/FAIL | |
| 4 | Drive to Tag 3 (Button) | | PASS/FAIL | |
| 5 | Drive to Tag 2 (1.0m Auto) | | PASS/FAIL | |
| 6 | Drive to Tag 3 (1.5m Auto) | | PASS/FAIL | |
| 7 | Multiple Distances (Tag 2) | | PASS/FAIL | |
| 8 | Multiple Distances (Tag 3) | | PASS/FAIL | |
| 9 | Vision Fusion Accuracy | | PASS/FAIL | |
| 10 | Edge Cases | | PASS/FAIL | |

### Performance Metrics

| Metric | Target | Actual | Notes |
|--------|--------|--------|-------|
| Tag detection range | 5.0m | | |
| Position accuracy | ±0.1m | | |
| Angle accuracy | ±2° | | |
| Vision update rate | >10 Hz | | |
| Phase transition time | <1s | | |

---

## 7. Next Steps After Testing

1. **Tune PID Gains**: Based on test results, adjust PID controller gains for better performance
2. **Optimize Distances**: Determine optimal approach distances for different scenarios
3. **Add More Test Autos**: Create additional autonomous routines for different use cases
4. **Document Best Practices**: Record optimal starting positions and conditions
5. **Performance Optimization**: Fine-tune vision fusion parameters based on accuracy measurements

---

## 8. Quick Reference

### Button Shortcuts
- **X + Y**: Drive to Tag 2 (simple)
- **Y + B**: Drive to Tag 3 (vision-assisted)
- **Start**: Drive to Tag 2 (simple)
- **A**: Cancel all commands

### Auto Chooser Options
- "Vision Test: Tag 2 (0.5m)"
- "Vision Test: Tag 2 (1.0m)"
- "Vision Test: Tag 2 (1.5m)"
- "Vision Test: Tag 2 (2.0m)"
- "Vision Test: Tag 3 (0.5m)"
- "Vision Test: Tag 3 (1.0m)"
- "Vision Test: Tag 3 (1.5m)"
- "Vision Test: Tag 3 (2.0m)"
- "Vision Test: Tag 2 (Vision-Assisted)"
- "Vision Test: Tag 3 (Vision-Assisted)"
- "Vision Test: Tag 2 (Simple PID)"

### Pose Reset Utility
- **Widget**: "TEST: Reset Robot Pose" (dropdown chooser)
- **Button**: "TEST: Reset Pose to Selected"
- **Available Positions**: Home, Field Center, Origin, Blue/Red start positions

### Key Constants
- `TARGET_DISTANCE_METERS` (VisionAssisted): 0.5m
- `VISION_SWITCH_DISTANCE`: 1.5m
- `POSITION_TOLERANCE`: 0.05m (VisionAssisted), 0.5m (DriveToAprilTag2)
- `MAX_APRILTAG_DISTANCE_METERS`: 5.0m

---

---

## 9. Pose Reset Utility Reference

### 9.1 Quick Guide

The pose reset utility allows you to quickly reset the robot's position on the field during testing, ensuring consistent starting conditions for repeatable tests.

**Access**: Shuffleboard/Glass → "TEST: Reset Robot Pose" widget

**Usage**:
1. Select position from dropdown
2. Click "TEST: Reset Pose to Selected" button
3. Verify pose update in Field2d widget

### 9.2 Available Positions

| Position | Coordinates | Use Case |
|----------|-------------|----------|
| **Home Position** | (3.0, 3.0, 0°) | Default safe starting position |
| **Field Center** | (varies by field) | Center of field testing |
| **Origin** | (0, 0, 0°) | Field origin reference |
| **Blue Start Left** | (1.5, 5.5, 0°) | Blue alliance left starting position |
| **Blue Start Center** | (1.5, 4.5, 0°) | Blue alliance center starting position |
| **Blue Start Right** | (1.5, 3.5, 0°) | Blue alliance right starting position |
| **Red Start Left** | (14.5, 5.5, 180°) | Red alliance left starting position |
| **Red Start Center** | (14.5, 4.5, 180°) | Red alliance center starting position |
| **Red Start Right** | (14.5, 3.5, 180°) | Red alliance right starting position |

### 9.3 Testing Workflow with Pose Reset

**Recommended Testing Workflow:**
1. **Setup**: Reset robot pose to a known starting position (e.g., "Home Position")
2. **Test**: Run autonomous command or button test
3. **Measure**: Record results (final position, accuracy, etc.)
4. **Reset**: Reset pose back to starting position
5. **Repeat**: Run same test again for repeatability verification

**Benefits:**
- Ensures consistent starting conditions
- Enables repeatable test results
- Saves time (no manual robot positioning)
- Allows testing from multiple starting positions easily

### 9.4 Troubleshooting Pose Reset

**Issue**: Pose reset button doesn't work
- **Solution**: Check that robot is enabled and drivetrain is functional
- Verify widget is visible on dashboard

**Issue**: Robot pose doesn't update in Field2d widget
- **Solution**: Check console for `[PoseResetTest]` confirmation message
- Verify drivetrain pose estimator is running
- Check for any errors in console

**Issue**: Selected position seems incorrect
- **Solution**: Verify field layout matches 2024 Crescendo
- Check that coordinates match expected field positions
- Use Field2d widget to visually verify position

---

**Document Version**: 1.1  
**Last Updated**: 2025  
**Author**: Test Plan Generator  
**Changes in v1.1**: Added Pose Reset Test Utility documentation

