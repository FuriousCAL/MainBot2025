# Swerve Drive Debugging Guide

## Problem Analysis

Your encoder readings have changed significantly from the original values:
- **Front Left (ID 7)**: Was -0.281250, Now 0.499756
- **Back Left (ID 9)**: Was 0.013184, Now -0.000244  
- **Back Right (ID 11)**: Was -0.051758, Now -0.999512
- **Front Right (ID 13)**: Was 0.170410, Now 0.500244

## Root Causes

1. **Encoder offsets are wrong** - Offsets don't match current wheel positions
2. **Possible inversion issues** - Wheels may be pointing wrong direction during rotation
3. **Wheels may have moved** - Physical alignment may have changed

## Step-by-Step Fix Process

### STEP 1: Physically Align All Wheels Forward
**CRITICAL:** Before taking any readings, you MUST physically align all wheels to point straight forward.

1. Power off the robot
2. Manually rotate each wheel so it points straight forward (aligned with robot's forward direction)
3. Use a straight edge or string to verify all 4 wheels are parallel and pointing forward
4. Mark the forward direction on your robot for reference

### STEP 2: Take New Encoder Readings
With wheels aligned forward and robot powered on:

1. Connect to Phoenix Tuner X
2. Read the "Sensor" or "Feedback" position for each steer motor:
   - ID 7 (Front Left Steer)
   - ID 9 (Back Left Steer)
   - ID 11 (Back Right Steer)
   - ID 13 (Front Right Steer)

### STEP 3: Calculate New Offsets
For each module, the offset = **negative of the reading when wheels are forward**

Example:
- If Front Left reads 0.5 rotations when forward → Offset = -0.5
- If Back Right reads -1.0 rotations when forward → Offset = 1.0

### STEP 4: Update Code with New Offsets
We'll update the offsets in TunerConstants.java

### STEP 5: Test and Debug Inversions
After updating offsets, test:
1. Command all wheels to 0° (forward) - they should all point forward
2. Command rotation - wheels should form an X pattern (front left + back right point right, front right + back left point left)
3. If wheels point wrong direction, we may need to flip inversions

## Inversion Troubleshooting

### If wheels point backwards when commanded forward:
- Try flipping the encoder inversion for that module
- OR try flipping the steer motor inversion

### If wheels point wrong direction during rotation:
- Check if left/right side inversions need to be flipped
- The pattern should be: Front Left + Back Right point one way, Front Right + Back Left point opposite

## Current Status Based on Your Readings

Based on your current readings (assuming wheels are NOT aligned):
- Front Left: 0.499756 → Needs offset of ~-0.5 when aligned
- Back Left: -0.000244 → Needs offset of ~0.0 when aligned  
- Back Right: -0.999512 → Needs offset of ~1.0 when aligned
- Front Right: 0.500244 → Needs offset of ~-0.5 when aligned

**BUT:** We need readings taken WHEN WHEELS ARE PHYSICALLY ALIGNED FORWARD to get correct offsets.

## Next Steps

1. ✅ Physically align all wheels forward
2. ✅ Take new encoder readings
3. ✅ Provide new readings to update offsets
4. ✅ Test and adjust inversions if needed

