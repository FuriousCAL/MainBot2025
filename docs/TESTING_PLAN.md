# MinBot2 Testing Plan & Controls Guide

## Controller Mappings (Driver - Port 0)

| Button / Axis | Action | Description |
| :--- | :--- | :--- |
| **Left Stick** | Drive | Controls robot translation (X/Y) |
| **Right Stick** | Rotate | Controls robot rotation |
| **LB** | Toggle Drive Mode | Switches between Field-Centric and Robot-Centric drive |
| **RB** | Brake | Hold to lock wheels in X pattern (prevent movement) |
| **A** | Cancel / Stop | Cancels active auto commands and stops the robot |
| **B** | Precision Align | Point wheels towards direction of Left Stick without driving |
| **Y** | Return Home | Autonomous drive to safe "Home" position (3m, 3m, 0°) |
| **Start** | Test: Vision Tag 2 | Autonomous drive to AprilTag 2 using simple PID (Test) |
| **Back** | Emergency Home | Backup button to return to Home position |
| **D-Pad Up** | Return Home | Shortcut for Home position |
| **D-Pad Down** | Test: Vision Tag 2 | Shortcut for Vision Test |
| **D-Pad Left** | Debug: 0° Alignment | Points all wheels to 0° (Forward) for encoder check |
| **D-Pad Right** | Debug: 90° Alignment | Points all wheels to 90° (Left) for encoder check |

### Vision Combinations (Hold first button, press second)
| Combination | Action | Description |
| :--- | :--- | :--- |
| **X + Y** | Go to Tag 2 | Speaker (Center) - Main Scoring Position |
| **X + B** | Go to Tag 1 | Blue Source Side |
| **Y + B** | Go to Tag 3 | Red Speaker Side |
| **X + A** | Go to Tag 4 | Red Source Side |

---

## Testing Plan

### Phase 1: Simulation Verification
**Objective:** Verify logic and coordinate systems without hardware risk.

1.  **Drive Reset:**
    *   Start Simulation.
    *   Drive robot away from origin.
    *   **Test:** Use "TEST: Reset Pose to Selected" on Dashboard to reset to "Home Position".
    *   *Note: In simulation, this may only reset rotation due to API limitations.*

2.  **Field-Centric Drive:**
    *   Rotate robot 90°.
    *   Push Left Stick "Up" (Forward).
    *   **Verify:** Robot moves "North" on the field (up on screen), not relative to its front.

3.  **Vision Auto-Drive (The Fix Verification):**
    *   Reset robot to (0,0) facing 0°.
    *   Press **Start** button (Drive to Tag 2).
    *   **Verify:** Robot drives towards Tag 2.
    *   **Edge Case:** Reset robot to (0,0) facing 90°.
    *   Press **Start** button.
    *   **Verify:** Robot drives towards Tag 2 (correcting for rotation), does NOT drive off-field.

### Phase 2: Hardware Sanity Checks (Blocks)
**Objective:** Ensure motors and encoders are configured correctly.

1.  **Vision Integration (See: `docs/VISION_TESTING_GUIDE.md`):**
    *   **CRITICAL**: Since the robot doesn't move on blocks, vision automations will fail (spin forever) unless you "move the world" (the tag).
    *   Test "Drive to Tag" by holding a tag and moving it relative to the suspended robot.

2.  **Wheel Alignment:**
    *   Put robot on blocks.
    *   Press **D-Pad Left**. Verify all wheels point straight forward.
    *   Press **D-Pad Right**. Verify all wheels point left.
    *   If inverted, check `TunerConstants.java` encoder offsets.

2.  **Drive Direction:**
    *   Push Left Stick Forward. Verify all wheels spin "forward".
    *   Push Left Stick Left. Verify all wheels spin to move left.
    *   Rotate Right Stick. Verify wheels align for rotation.

### Phase 3: Field Testing (Low Speed)
**Objective:** Verify vision and autonomous movement.

1.  **Brake Test:**
    *   Drive slowly.
    *   Press **RB**. Robot should stop immediately and lock wheels.

2.  **Home Command:**
    *   Drive to random spot.
    *   Press **Y**.
    *   Robot should autonomously navigate to (3,3). Be ready to press **A** (Cancel) if it misbehaves.

3.  **Vision Approach:**
    *   Place robot near Tag 2 (within 2-3 meters).
    *   Press **Start** or **X+Y**.
    *   Robot should align and approach the tag.
