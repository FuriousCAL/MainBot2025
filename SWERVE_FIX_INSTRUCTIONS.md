# Swerve Drive Fix Instructions

## Current Status
- ✅ Updated encoder offsets based on your latest readings
- ⚠️ Need to test and verify
- ⚠️ May need to adjust inversion settings

## Updated Offsets
Based on your latest readings, offsets have been updated to:
- **Front Left (ID 7)**: 0.499756 rotations
- **Front Right (ID 13)**: 0.500244 rotations  
- **Back Left (ID 9)**: -0.000244 rotations (essentially 0)
- **Back Right (ID 11)**: -0.999512 rotations (≈ -1.0, equivalent to 0.0)

## Testing Procedure

### Step 1: Deploy and Test Forward Alignment
1. Deploy the updated code to your robot
2. Power on the robot
3. Use your driver station to command all wheels to 0° (forward)
4. **Observe**: Do all wheels point straight forward?
   - ✅ **YES** → Offsets are correct! Proceed to Step 2
   - ❌ **NO** → Go to "Offset Troubleshooting" below

### Step 2: Test Rotation
1. Command the robot to rotate in place (spin around its center)
2. **Observe**: Do the wheels form an "X" pattern?
   - Front Left + Back Right should point in one direction
   - Front Right + Back Left should point in the opposite direction
   - ✅ **YES** → Inversions are correct! You're done!
   - ❌ **NO** → Go to "Inversion Troubleshooting" below

## Offset Troubleshooting

### If wheels are NOT pointing forward when commanded to 0°:

**Option A: Try Opposite Sign**
If wheels are off by ~180° or pointing wrong direction, we may need to flip the offset sign.

Current offsets use: `offset = reading`
Try: `offset = -reading`

**Option B: Take New Readings**
1. **Physically align all wheels to point straight forward**
2. Power on robot (keep wheels aligned)
3. Take NEW encoder readings from Phoenix Tuner
4. Update offsets: `offset = reading` (or `offset = -reading` if Option A didn't work)

### Quick Offset Adjustment Formula
If a wheel is off by a small angle:
- 1° = 0.00278 rotations
- If wheel is 5° clockwise from forward: subtract 0.014 from offset
- If wheel is 5° counter-clockwise from forward: add 0.014 to offset

## Inversion Troubleshooting

### Problem: Wheels point wrong direction during rotation

#### Symptoms:
- Wheels don't form X pattern during rotation
- Robot rotates in wrong direction
- Wheels fight each other during rotation

#### Fix Options (try in order):

**Option 1: Flip Left/Right Side Inversions**
In `TunerConstants.java`, try changing:
```java
private static final boolean kInvertLeftSide = false;  // was true
private static final boolean kInvertRightSide = false; // was true
```

**Option 2: Flip Individual Module Inversions**
Try flipping encoder inversion for modules that point wrong:
```java
// For a module pointing wrong direction:
private static final boolean kFrontLeftEncoderInverted = false; // was true
```

**Option 3: Flip Steer Motor Inversions**
Try flipping steer motor inversion:
```java
// For a module pointing wrong direction:
private static final boolean kFrontLeftSteerMotorInverted = true; // was false
```

### Expected Behavior During Rotation:
- **Front Left**: Points right (when viewed from top)
- **Front Right**: Points left (when viewed from top)
- **Back Left**: Points left (when viewed from top)
- **Back Right**: Points right (when viewed from top)

This creates an "X" pattern allowing the robot to rotate in place.

## Current Inversion Settings

```java
// Side inversions
kInvertLeftSide = true
kInvertRightSide = true

// Individual module inversions
Front Left:  Steer Motor = false, Encoder = true
Front Right: Steer Motor = false, Encoder = true
Back Left:   Steer Motor = false, Encoder = true
Back Right:  Steer Motor = false, Encoder = true
```

## Debugging Checklist

- [ ] Deploy updated code
- [ ] Test: Command wheels to 0° → All point forward?
- [ ] Test: Command rotation → Wheels form X pattern?
- [ ] Test: Drive forward → Robot moves forward (no strafe)?
- [ ] Test: Strafe left → Robot moves left (no rotation)?
- [ ] Test: Strafe right → Robot moves right (no rotation)?

## If Nothing Works

1. **Verify Physical Alignment**: Ensure wheels are actually aligned forward when you take readings
2. **Check Module Positions**: Verify module X/Y positions are correct (should be 11.875" from center)
3. **Verify Gear Ratios**: Check that drive/steer gear ratios match your hardware
4. **Check CANcoder IDs**: Ensure encoder IDs (3, 4, 5, 6) match motor IDs (7, 9, 11, 13)
5. **Factory Reset**: Try factory resetting all motors again and retake readings

## Next Steps

1. ✅ Deploy the updated code with new offsets
2. ✅ Test forward alignment (Step 1)
3. ✅ Test rotation (Step 2)
4. ✅ Adjust offsets/inversions as needed based on results
5. ✅ Report back with results for further debugging if needed

## Important Notes

- **Offset Sign Convention**: CTRE Phoenix 6 typically uses: `corrected_angle = encoder_reading - offset`
- If this doesn't work, try: `corrected_angle = offset - encoder_reading` (flip all offset signs)
- **Normalization**: Encoder readings are normalized to [0, 1) range, so -0.999512 ≈ 0.000488
- **Physical Alignment**: Always physically align wheels before taking offset readings

