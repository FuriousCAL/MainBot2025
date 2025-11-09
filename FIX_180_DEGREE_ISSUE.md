# Fixing the 180° Issue - Systematic Approach

## Problem Summary

**Symptoms:**
- Command: 0° (forward) via D-pad Left
- Robot receives:
  - FL and FR: **180°** (WRONG - pointing backward)
  - BL and BR: **0°** (CORRECT - pointing forward)
- Physical alignment:
  - FL and FR: ~90° off
  - BL and BR: ~45° off

## Root Cause Analysis

The 180° issue suggests one of these problems:

### Possibility 1: Coordinate System Flipped
- CTRE might use **X positive = BACKWARD** (not forward)
- If so, front modules at (11.875, 11.875) are actually at the BACK
- To face forward (negative X), they need to point 180°

### Possibility 2: Encoder Offset Sign Wrong
- We just flipped all offset signs (TEST in code)
- Might need to flip back AND adjust encoder inversions

### Possibility 3: Module Position Signs Wrong
- Front modules might need **negative X** instead of positive
- Or coordinate system interpretation is different

---

## Fix Strategy: Try in Order

### FIX 1: Try Negative Offset Signs (ALREADY DONE)
✅ All offsets flipped to negative of original readings
- FL: -0.303223 (was 0.303223)
- FR: 0.442627 (was -0.442627)
- BL: -0.052246 (was 0.052246)
- BR: 0.042969 (was -0.042969)

**Test this first!**

---

### FIX 2: Flip Front Module X Positions

If FIX 1 doesn't work, try flipping X positions for front modules:

**Current:**
- FL: X = 11.875, Y = 11.875
- FR: X = 11.875, Y = -11.875

**Try:**
- FL: X = -11.875, Y = 11.875
- FR: X = -11.875, Y = -11.875

This assumes CTRE uses X positive = backward.

---

### FIX 3: Flip Encoder Inversions for Front Modules

If FIX 2 doesn't work, try flipping encoder inversions:

**Current:**
- FL: Encoder Inverted = true
- FR: Encoder Inverted = true

**Try:**
- FL: Encoder Inverted = false
- FR: Encoder Inverted = false

---

### FIX 4: Fine-Tune Individual Offsets

Once the 180° issue is fixed, fine-tune each wheel:

1. **Command wheel to 0°**
2. **Measure error** (degrees)
3. **Adjust offset**: `new_offset = old_offset ± (error_degrees / 360.0)`
   - If wheel points **clockwise** from forward: **subtract** error
   - If wheel points **counter-clockwise** from forward: **add** error

---

## Testing Procedure

### Test 1: Deploy Negative Offsets
1. Deploy code with flipped offset signs (already done)
2. Hold D-pad Left (command 0°)
3. **Check**: Are FL and FR still receiving 180°?
   - ✅ **NO** (they receive 0° now): Good! Proceed to fine-tune
   - ❌ **YES** (still 180°): Try FIX 2 (flip X positions)

### Test 2: Check Physical Alignment
1. After deploying FIX 1, check if wheels are closer to forward
2. Measure error for each wheel:
   - FL: ___ degrees (clockwise/counter-clockwise)
   - FR: ___ degrees (clockwise/counter-clockwise)
   - BL: ___ degrees (clockwise/counter-clockwise)
   - BR: ___ degrees (clockwise/counter-clockwise)

### Test 3: Fine-Tune Offsets
1. For each wheel, adjust offset based on error
2. Deploy and test again
3. Repeat until all wheels point forward at 0°

---

## Quick Reference: Offset Adjustment

```
Error in rotations = Error in degrees / 360.0

If wheel points CLOCKWISE from forward:
  New Offset = Old Offset - Error

If wheel points COUNTER-CLOCKWISE from forward:
  New Offset = Old Offset + Error
```

### Examples:

**Front Left:**
- Commanded: 0°
- Actual: 45° clockwise
- Error: 45° / 360° = 0.125 rotations
- Current offset: -0.303223
- New offset: -0.303223 - 0.125 = **-0.428223**

**Back Right:**
- Commanded: 0°
- Actual: 30° counter-clockwise
- Error: 30° / 360° = 0.083 rotations
- Current offset: 0.042969
- New offset: 0.042969 + 0.083 = **0.125969**

---

## Current Status

✅ **FIX 1 Applied**: All offset signs flipped
⏳ **Testing Needed**: Deploy and test if 180° issue is resolved
⏳ **Fine-Tuning**: Adjust individual offsets based on errors

---

## Next Steps

1. **Deploy FIX 1** (negative offsets - already in code)
2. **Test D-pad Left** - check if FL/FR still receive 180°
3. **Report results**:
   - Do FL/FR still receive 180°? (YES/NO)
   - What error angles do you see for each wheel?
4. **Apply next fix** based on results

---

## Expected Outcome

After fixes:
- ✅ All wheels receive **0°** when D-pad Left pressed
- ✅ All wheels point **forward** (aligned with robot front)
- ✅ Error less than 5° for each wheel
- ✅ Rotation creates correct tangential pattern

