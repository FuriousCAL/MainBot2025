# Systematic Wheel Alignment Fix

## Problem Analysis

**Symptoms:**
- D-pad Left sends 0°, but robot receives:
  - FL and FR: 180° (WRONG - should be 0°)
  - BL and BR: 0° (CORRECT)
- Physical misalignment:
  - FL and FR: ~90° off
  - BL and BR: ~45° off

**Root Causes:**
1. **Offset sign may be inverted** - CTRE might need negative offsets
2. **Encoder inversion may be wrong** - All encoders are inverted, might need to flip
3. **Coordinate system mismatch** - 180° offset suggests coordinate system issue

---

## Step-by-Step Fix Process

### STEP 1: Try Flipping Offset Signs

CTRE Phoenix 6 might use: `corrected_angle = offset - encoder_reading` instead of `encoder_reading - offset`

**Current offsets:**
- FL: 0.303223
- FR: -0.442627
- BL: 0.052246
- BR: -0.042969

**Try negative offsets:**
- FL: -0.303223
- FR: 0.442627
- BL: -0.052246
- BR: 0.042969

---

### STEP 2: Test Each Wheel Individually

We'll add individual wheel test buttons so you can test one wheel at a time.

---

### STEP 3: Adjust Offsets Based on Error

For each wheel:
1. Command wheel to 0°
2. Measure physical error (degrees)
3. Adjust offset: `new_offset = old_offset + (error_degrees / 360.0)`

---

### STEP 4: Fix Inversions

After offsets are correct, test rotation pattern and adjust inversions if needed.

---

## Testing Procedure

### Test 1: Check Offset Sign
1. Deploy code with **negative offsets**
2. Hold D-pad Left (command 0°)
3. Check if wheels are closer to forward
4. If better, we're on the right track
5. If worse, try flipping encoder inversions instead

### Test 2: Individual Wheel Testing
1. Use individual wheel buttons (we'll add these)
2. Test each wheel one at a time
3. Adjust offsets individually
4. Note which wheels are correct vs wrong

### Test 3: Fine-Tune Offsets
1. For each misaligned wheel:
   - Command to 0°
   - Measure error angle
   - Adjust offset: `error_rotations = error_degrees / 360.0`
   - If wheel points clockwise from forward: **subtract** from offset
   - If wheel points counter-clockwise from forward: **add** to offset

### Test 4: Verify Rotation Pattern
1. Once all wheels align to 0° correctly
2. Test rotation (right stick X)
3. Verify tangential pattern (not X pattern)
4. Adjust inversions if needed

---

## Quick Fix: Try Negative Offsets First

This is the fastest test - let's try flipping all offset signs and see if that fixes the 180° issue.

---

## Individual Wheel Offset Adjustment Formula

If a wheel is off by X degrees when commanded to 0°:

```
Error in rotations = X degrees / 360.0

New Offset = Old Offset ± Error

If wheel points CLOCKWISE from forward: New Offset = Old Offset - Error
If wheel points COUNTER-CLOCKWISE from forward: New Offset = Old Offset + Error
```

### Example:
- Wheel commanded to 0°
- Wheel actually points 45° clockwise
- Error = 45° / 360° = 0.125 rotations
- Old offset = 0.303223
- New offset = 0.303223 - 0.125 = 0.178223

---

## Priority Order

1. **Fix offset signs** (try negative first)
2. **Fix FL and FR** (they're 180° off, highest priority)
3. **Fix BL and BR** (they're 45° off)
4. **Test rotation pattern**
5. **Fine-tune inversions if needed**

---

## Expected Behavior After Fix

✅ All wheels point forward when D-pad Left pressed  
✅ Robot receives 0° for all wheels (not 180° for some)  
✅ Rotation creates tangential pattern (not X pattern)  
✅ Wheels stay aligned during driving  

