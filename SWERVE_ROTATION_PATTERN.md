# Correct Swerve Wheel Pattern for Rotation

## ❌ WRONG: X Pattern (Wheels Point Inward)
```
        FL ←     → FR
           \   /
            \ /
             X
            / \
           /   \
        BL →     ← BR
```
**This is INCORRECT** - wheels pointing toward center will fight each other!

---

## ✅ CORRECT: Tangential Pattern (Wheels Point Perpendicular to Center)

For a **clockwise rotation** (robot spins clockwise when viewed from top):

```
        FL →     ← FR
           |   |
           |   |
           |   |
        BL ←     → BR
```

### Detailed Explanation:

**Front Left (FL)**: Points **RIGHT** (→)
- Located at front-left corner
- To rotate robot clockwise, needs to push robot to the right
- Points perpendicular to line from center to FL position

**Front Right (FR)**: Points **LEFT** (←)
- Located at front-right corner  
- To rotate robot clockwise, needs to push robot to the left
- Points perpendicular to line from center to FR position

**Back Left (BL)**: Points **LEFT** (←)
- Located at back-left corner
- To rotate robot clockwise, needs to push robot to the left
- Points perpendicular to line from center to BL position

**Back Right (BR)**: Points **RIGHT** (→)
- Located at back-right corner
- To rotate robot clockwise, needs to push robot to the right
- Points perpendicular to line from center to BR position

---

## Visual Diagram (Top View)

```
                    FRONT
                     ↑
                     |
          FL →  |  ← FR
                |    
                |  Robot Center
                |    
          BL ←  |  → BR
                     |
                     ↓
                    BACK
```

### Key Points:
- **FL and BR** point in the **SAME direction** (right →)
- **FR and BL** point in the **SAME direction** (left ←)
- All wheels are **perpendicular** to lines from center to wheel
- This creates a **pure rotation** with no translation

---

## For Counter-Clockwise Rotation

Simply reverse the directions:
- FL points LEFT (←)
- FR points RIGHT (→)
- BL points RIGHT (→)
- BR points LEFT (←)

---

## How to Verify Your Robot

### Test Procedure:
1. **Command rotation** using right joystick (push right = clockwise)
2. **Observe wheel directions** from above
3. **Expected pattern for clockwise rotation**:
   - Front Left: Points right
   - Front Right: Points left
   - Back Left: Points left
   - Back Right: Points right

### If Pattern is Wrong:

#### Problem: Wheels form X pattern (point inward)
**Solution**: Inversion settings are likely wrong. Try:
- Flip `kInvertLeftSide` or `kInvertRightSide`
- OR flip individual module encoder inversions

#### Problem: Wheels point wrong directions
**Solution**: 
- Check if FL/BR point same direction (they should)
- Check if FR/BL point same direction (they should)
- If pairs don't match, adjust inversions for those modules

#### Problem: Robot doesn't rotate (wheels fight each other)
**Solution**:
- This usually means wheels are pointing toward center (X pattern)
- Flip side inversions or module inversions

---

## Mathematical Explanation

For a wheel at position (x, y) relative to robot center:
- **Wheel angle** = `atan2(-x, y)` for clockwise rotation
- This makes the wheel point **perpendicular** to the radius vector

For your robot with modules at ±11.875" from center:
- Front Left (11.875, 11.875): angle ≈ 45° (points right/forward-right)
- Front Right (11.875, -11.875): angle ≈ -45° (points left/forward-left)
- Back Left (-11.875, 11.875): angle ≈ 135° (points left/back-left)
- Back Right (-11.875, -11.875): angle ≈ -135° (points right/back-right)

Note: These are approximate - the exact angles depend on your coordinate system, but the pattern should be that **diagonal wheels point the same direction**.

---

## Summary

✅ **CORRECT**: Diagonal wheels (FL+BR, FR+BL) point same direction  
❌ **WRONG**: Wheels point inward toward center (X pattern)  
❌ **WRONG**: All wheels point same direction (translation, not rotation)  
❌ **WRONG**: Wheels point outward from center  

The key is: **Each wheel points perpendicular to the line from robot center to that wheel's position**, creating a pure rotation.

