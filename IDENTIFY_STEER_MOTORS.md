# How to Identify Steer Motors vs Drive Motors

## Quick Reference: Motor ID Mapping

Based on your current configuration:

### Front Left Module
- **Steer Motor**: ID **7** (rotates the wheel module)
- **Drive Motor**: ID **8** (spins the wheel)
- **CANcoder**: ID **6** (absolute encoder for steering)

### Front Right Module
- **Steer Motor**: ID **13**
- **Drive Motor**: ID **14**
- **CANcoder**: ID **5**

### Back Left Module
- **Steer Motor**: ID **9**
- **Drive Motor**: ID **10**
- **CANcoder**: ID **3**

### Back Right Module
- **Steer Motor**: ID **11**
- **Drive Motor**: ID **12**
- **CANcoder**: ID **4**

---

## Physical Identification Methods

### 1. **Position on Module**
- **Steer Motor**: Usually mounted **vertically** or on top of the module
  - Connected to the swivel/rotation mechanism
  - Has a gear that rotates the entire wheel assembly
  - Typically smaller/shorter than drive motor
  
- **Drive Motor**: Usually mounted **horizontally** or on the side
  - Connected directly to the wheel via gears/belt
  - Spins the wheel for forward/backward motion
  - Typically larger/longer than steer motor

### 2. **Gear Ratio**
- **Steer Motor**: Higher gear ratio (21.43:1 in your config)
  - More rotations per output rotation
  - Better for precise positioning
  
- **Drive Motor**: Lower gear ratio (6.75:1 in your config)
  - Fewer rotations per output rotation
  - Better for speed

### 3. **CANcoder Connection**
- **Steer Motor**: Has a **CANcoder** attached nearby
  - CANcoder provides absolute position feedback
  - Typically ID is close to steer motor ID (e.g., Steer ID 7 → CANcoder ID 6)

---

## Software Identification Methods

### Method 1: Phoenix Tuner X - Check Encoder Feedback

1. Open Phoenix Tuner X
2. Select a motor (e.g., ID 7)
3. Check the **"Sensor" or "Feedback"** position
4. **Steer Motor Behavior**:
   - Position changes when you **rotate the wheel module** manually
   - Position stays in range 0-1 rotations (normalized)
   - Reading changes even when wheel isn't spinning
   
5. **Drive Motor Behavior**:
   - Position changes when you **spin the wheel** manually
   - Position can be hundreds/thousands of rotations
   - Reading only changes when wheel spins

### Method 2: Test with D-Pad Left Button

1. Deploy code with the new D-pad Left button (points wheels to 0°)
2. Hold D-pad Left
3. **Steer Motor**: Will rotate the wheel module to point forward
4. **Drive Motor**: Will not move (wheels should not spin)

### Method 3: Check Current Draw

1. In Phoenix Tuner, monitor current
2. **Steer Motor**: 
   - Low current when stationary
   - Current spikes when rotating module
   - Typically 60A limit (your config)
   
3. **Drive Motor**:
   - Can draw high current when spinning wheel
   - Typically 80A stator limit (your config)

### Method 4: Manual Rotation Test

**Test Steer Motor (ID 7, 9, 11, 13)**:
1. Power on robot
2. Manually rotate the **wheel module** (not the wheel itself)
3. Watch Phoenix Tuner - the steer motor's position should change
4. The wheel should rotate around its vertical axis

**Test Drive Motor (ID 8, 10, 12, 14)**:
1. Power on robot
2. Manually **spin the wheel** (not rotate the module)
3. Watch Phoenix Tuner - the drive motor's position should change
4. The wheel should spin like a car wheel

---

## Verification Checklist

Use this to verify your motor assignments:

### For Each Module:

- [ ] **Steer Motor** rotates the wheel module when commanded
- [ ] **Steer Motor** has CANcoder feedback that changes when module rotates
- [ ] **Drive Motor** spins the wheel when commanded
- [ ] **Drive Motor** position changes when wheel spins manually
- [ ] **CANcoder ID** is close to steer motor ID (within 1-2)
- [ ] **Steer Motor** has higher gear ratio (21.43:1)
- [ ] **Drive Motor** has lower gear ratio (6.75:1)

---

## Common Issues

### Issue: Can't tell which is which
**Solution**: 
1. Use Phoenix Tuner to manually rotate module → Steer motor position changes
2. Use Phoenix Tuner to manually spin wheel → Drive motor position changes

### Issue: Motor IDs don't match expected
**Solution**:
1. Check CAN bus connection
2. Verify motor IDs in Phoenix Tuner
3. Update `TunerConstants.java` if IDs are different

### Issue: Encoder readings don't make sense
**Solution**:
1. Verify CANcoder is connected to correct module
2. Check CANcoder ID matches configuration
3. Ensure CANcoder is on same CAN bus as motors

---

## Quick Test Procedure

1. **Power on robot**
2. **Open Phoenix Tuner X**
3. **Select Motor ID 7** (Front Left Steer)
4. **Manually rotate the Front Left wheel module** (rotate it around vertical axis)
5. **Observe**: 
   - ✅ If position changes → This is the steer motor
   - ❌ If position doesn't change → This might be the drive motor
6. **Repeat for other motors** to verify all assignments

---

## Your Current Configuration Summary

| Module | Steer Motor ID | Drive Motor ID | CANcoder ID |
|--------|----------------|----------------|-------------|
| Front Left | 7 | 8 | 6 |
| Front Right | 13 | 14 | 5 |
| Back Left | 9 | 10 | 3 |
| Back Right | 11 | 12 | 4 |

**Remember**: 
- Steer motors rotate the **module** (wheel pointing direction)
- Drive motors spin the **wheel** (forward/backward motion)
- CANcoders are attached to steer motors for absolute position feedback

