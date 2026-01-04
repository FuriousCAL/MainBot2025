# Vision System Testing Guide

This guide covers best practices for testing the Vision System integration, specifically when the robot is on blocks or in a lab setting.

## 1. On Blocks Testing (Safety Mode)

**Scenario**: Robot is elevated on blocks. Wheels mimic movement, but robot is stationary in 3D space.

### Expected Behavior
*   **Odometry**: When you command a drive, the encoders will count up, and the "Odometry Pose" will move on the dashboard map.
*   **Vision Pose**: Since the camera isn't physically moving, the "Vision Pose" will remain static (unless you move the AprilTag).
*   **Pose Estimator Conflict**: The robot's fused pose will try to move with the wheels (odometry) but get "pulled back" by the vision measurements saying "I haven't moved." You might see the robot icon jitter or snap back. **This is normal behavior on blocks.**

### "Moving the World" Technique
To test vision automations (e.g., "Drive to AprilTag") on blocks, you must **move the AprilTag** instead of the robot.

1.  **Setup**: Hold a printed AprilTag (e.g., Tag ID 2) in front of the camera.
2.  **Enable**: Enable Teleop/Auto.
3.  **Command**: Trigger "Drive to AprilTag 2".
4.  **Test**:
    *   **Distance**: Walk the tag *away* from the robot. The wheels should spin faster (trying to catch up). Walk the tag *closer*. The wheels should slow down or reverse.
    *   **Rotation**: Move the tag left/right. The robot should attempt to rotate to face the tag.
    *   **Loss**: Hide the tag. The robot should stop or switch to odometry-only mode (checking `Vision/HasTargets`).

## 2. Static Accuracy Check

Before driving, verify the Vision System is calibrated.

1.  Place the robot at a known location (e.g., exactly 2 meters from a tag, facing it).
2.  Check `Shuffleboard > Vision > Estimated Pose`.
3.  **Pass Criteria**:
    *   X/Y matches reality within ~5cm (2 inches).
    *   Rotation matches within ~2 degrees.
    *   `Tags Used` is > 0.
4.  **Troubleshooting**:
    *   If off by meters: Check `Camera To Robot Transform` constants in code.
    *   If off by rotation: Check if Roll/Pitch/Yaw are swapped or inverted.

## 3. Latency & Robustness

1.  **Occlusion**: Briefly cover the camera. 
    *   Expect: `Has Targets` -> False. `Connected` -> True.
    *   Uncover. `Has Targets` -> True (simultaneously).
2.  **Latency**: Wave the tag quickly.
    *   Check `Vision > Log > Latency` in AdvantageScope. Standard is 15-50ms. If >100ms, driving will be oscillatory.

## 4. Automation Status Indicators

Watch the `Vision` tab on Shuffleboard:
*   **Connected**: Must be `TRUE`. If false, check Ethernet/NetworkTables.
*   **Has Targets**: Must be `TRUE` for vision-updates.
*   **Best Target Ambiguity**: Should be low (< 0.2). If >0.5, pose is unreliable (jumping).

## 5. Typical Failure Modes

*   **"Spinning in Circles"**: Usually Camera Yaw constant is inverted, or PID output is inverted.
*   **"Stops Early"**: Distance calculation is wrong, or "Target Area" logic is triggering early.
*   **"Jittery Driving"**: Latency is too high, or Vision Trust (Std Dev) is too high vs Odometry.
