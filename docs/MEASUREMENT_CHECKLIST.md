# Robot Physical Dimensions Measurement Checklist

## Critical Measurements for Swerve Drive Configuration

### 1. MODULE POSITIONS (Most Critical - Affects Odometry Accuracy)
**Measure from the center of the robot's rotation (typically the geometric center) to the center of each swerve module's wheel contact point.**

**Coordinate System:**
- **X-axis**: Positive = Forward (toward front of robot)
- **Y-axis**: Positive = Left (when facing forward)
- **Origin**: Center of robot rotation (typically geometric center)

**Measurements Needed:**
- [ ] **Front Left Module X Position** (inches): Distance from center forward/backward
- [ ] **Front Left Module Y Position** (inches): Distance from center left/right
- [ ] **Front Right Module X Position** (inches): Distance from center forward/backward
- [ ] **Front Right Module Y Position** (inches): Distance from center left/right
- [ ] **Back Left Module X Position** (inches): Distance from center forward/backward
- [ ] **Back Left Module Y Position** (inches): Distance from center left/right
- [ ] **Back Right Module X Position** (inches): Distance from center forward/backward
- [ ] **Back Right Module Y Position** (inches): Distance from center left/right

**How to Measure:**
1. Mark the center of your robot (geometric center or center of rotation)
2. Mark the center of each wheel's contact patch with the ground
3. Measure horizontal distance (X) and lateral distance (Y) for each module
4. Record measurements in inches (we'll convert to meters in code)

**Current Values in Code:**
- Front Left: X = -5", Y = 5"
- Front Right: X = 5", Y = 5"
- Back Left: X = 5", Y = 5"
- Back Right: X = -5", Y = -5"

---

### 2. WHEEL RADIUS (Affects Speed Calculations)
**Measure the radius of your drive wheels.**

- [ ] **Wheel Radius** (inches): Measure from wheel center to outer edge of wheel

**How to Measure:**
1. Measure the diameter of the wheel (or use manufacturer specs)
2. Divide by 2 to get radius
3. If measuring diameter: Record diameter = _____ inches, Radius = _____ inches

**Current Value in Code:** 2.0 inches (diameter would be 4 inches)

**Verification:** If you have 4" diameter wheels, radius = 2.0" ✓

---

### 3. ROBOT DIMENSIONS (For PathPlanner and Constants)
**Overall robot size measurements.**

- [ ] **Robot Length** (inches): Front to back (including bumpers)
- [ ] **Robot Width** (inches): Left to right (including bumpers)
- [ ] **Wheelbase Length** (inches): Distance between front and back module centers
- [ ] **Wheelbase Width** (inches): Distance between left and right module centers
- [ ] **Robot Mass** (lbs or kg): Total robot weight

**Current Values:**
- Constants.java: Wheelbase = 24" x 24" (0.6096m x 0.6096m)
- PathPlanner: Robot size = 0.38m x 0.38m (14.96" x 14.96")
- PathPlanner: Robot mass = 14.68 kg (32.4 lbs)
- Constants.java: Robot mass = 54.0 kg (120 lbs)

**Note:** These values should match between files!

---

### 4. GEAR RATIOS (Mechanical - Verify from Manufacturer)
**These are typically from manufacturer specifications, but verify if possible.**

- [ ] **Drive Gear Ratio**: Motor rotations per wheel rotation
- [ ] **Steer Gear Ratio**: Motor rotations per module rotation
- [ ] **Coupling Ratio**: Drive motor rotations per steer rotation

**Current Values in Code:**
- Drive Gear Ratio: 6.746031746031747:1
- Steer Gear Ratio: 21.428571428571427:1
- Coupling Ratio: 3.5714285714285716:1

**Verification:** Check your swerve module documentation/manufacturer specs

---

### 5. MAXIMUM SPEED AT 12V (Performance Tuning)
**This requires testing with the robot, but you can calculate theoretically first.**

- [ ] **Theoretical Max Speed** (m/s): Calculate or measure
- [ ] **Actual Max Speed** (m/s): Measure during testing (optional, for later)

**Calculation Formula:**
```
Max Speed (m/s) = (Motor Free Speed RPM / 60) × (2π × Wheel Radius in meters) / Drive Gear Ratio
```

**Current Value in Code:** 1.0 m/s (reduced for testing, original was 4.73 m/s)

**Note:** This will need to be tuned during testing. Start with calculated value.

---

### 6. BONUS: Robot Center Reference Point
**To ensure accurate measurements, define your robot's center point.**

- [ ] **Where is your robot's center?** (Geometric center, center of mass, or specific reference point?)
- [ ] **Does this match where you're measuring from?**

---

## Measurement Tips

1. **Use a tape measure** - More accurate than rulers for larger distances
2. **Measure to the center of the wheel contact patch** - Not the edge of the module
3. **Be consistent** - Use the same reference point for all measurements
4. **Measure twice** - Verify critical measurements
5. **Record units** - We're using inches for module positions, but code uses meters (we'll convert)

---

## Files That Need Updates

Once you have measurements, we'll update:
1. `TunerConstants.java` - Module positions (X, Y), wheel radius, max speed
2. `Constants.java` - Robot dimensions, wheelbase
3. `pathplanner/settings.json` - Module positions, robot dimensions, mass

---

## Quick Reference: Current vs. Expected

| Measurement | Current Value | Your Measurement |
|------------|---------------|------------------|
| Front Left X | -5" | _____ " |
| Front Left Y | 5" | _____ " |
| Front Right X | 5" | _____ " |
| Front Right Y | 5" | _____ " |
| Back Left X | 5" | _____ " |
| Back Left Y | 5" | _____ " |
| Back Right X | -5" | _____ " |
| Back Right Y | -5" | _____ " |
| Wheel Radius | 2.0" | _____ " |
| Robot Length | ~15-24" | _____ " |
| Robot Width | ~15-24" | _____ " |
| Wheelbase Length | 24" | _____ " |
| Wheelbase Width | 24" | _____ " |

---

**Ready to measure? Fill in the values above and we'll update all the code files!**

