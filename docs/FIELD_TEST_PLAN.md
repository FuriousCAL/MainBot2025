# MinBot2 Field Test Plan

## 1. Safety Configuration Check
**CRITICAL:** Before testing, check `src/main/java/frc/robot/constants/Constants.java`.

### Current Mode: `SAFETY_TESTING_MODE = true`
*   **Max Speed:** 0.5 m/s (Crawl speed)
*   **Max Rotation:** 180 deg/s
*   **Teleop Control:** Velocity Loop (Won't race on blocks)
*   **Timeouts:** 50s (Extended to allow slow travel)

### Competition Mode: `SAFETY_TESTING_MODE = false`
*   **Max Speed:** 4.5 m/s (Full speed)
*   **Max Rotation:** ~360 deg/s
*   **Teleop Control:** Open Loop (Responsive)
*   **Timeouts:** ~10s (Standard)

---

## 2. Controller Map (Driver - Port 0)

| Input | Function | Notes |
| :--- | :--- | :--- |
| **Left Stick** | Translate (Drive) | Forward/Back/Left/Right |
| **Right Stick** | Rotate | Turn Robot |
| **LB** | Toggle Field/Robot Centric | Default: Field Centric |
| **RB** | Brake Mode | Holds position while held |
| **Start** | **Velocity Test** | Drives fwd at fixed 0.5 m/s (Verify on blocks) |
| **Back** | Emergency Home | Resets pose to (3,3) |
| **Y** | Drive Home | Navigate to Safe Pose (3,3) |
| **A** | Cancel / Brake |cancels all commands |
| **B** | Point Wheels | Align wheels to stick direction |
| **POV Up** | **Go to Speaker** | Vision-Assisted (Blue 7 / Red 4) |
| **POV Left** | **Go to Amp** | Vision-Assisted (Blue 6 / Red 5) |
| **POV Right** | **Go to Source** | Vision-Assisted (Blue 1 / Red 10) |
| **POV Down** | **Auto Sequence** | Visits Amp -> S
| **X + A** | Vision Tag 4 | |

---

## 3. Pre-Flight Checklist

### Physical
- [ ] **Battery:** Battery Secure & Charged (>12.4V DS).
- [ ] **Ethernet:** Radio connected? (If using radio).
- [ ] **Bumpers:** Secure (if testing contact).
- [ ] **Blocks:** Is robot on blocks? (If executing initial spin up).

### Software (Shuffleboard)
- [ ] **Tab:** "Match Setup"
- [ ] **Alliance:** Select correct Alliance (Red/Blue).
- [ ] **Start Pose:** Select starting position.
- [ ] **Action:** Press "SET START POSE" button.
- [ ] **Tab:** "Drive"
- [ ] **Check:** "Current Pose" box updates when you move robot manually.

---

## 4. Test Procedures

### Phase 1: Basic Mobility (Teleop)
1.  Enable Teleop.
2.  Gently push Left Stick forward.
    *   *Expectation:* Robot wheels spin forward slowly (cap at 0.5 m/s).
3.  Rotate Right Stick.
    *   *Expectation:* Robot spins slowly.
4.  Test **Start Button** (Velocity Test).
    *   *Expectation:* Wheels spin at consistent 0.5 m/s steady hum.

### Phase 2: Vision & Automation (Teleop)
*Requires AprilTags to be visible*
1.  Point robot generally towards Speaker (Tag 7 Blue / 4 Red).
2.  Press **POV Up**.
    *   *Expectation:* Robot drives to Speaker. It may be slow (taking >10s).
    *   *Success:* Logs say "Target reached".
    *   *Failure:* Logs say "Global timeout" or "Vision phase timeout". 
3.  Test **POV Down** (Auto Sequence).
    *   *Expectation:* Robot drives to Amp, waits, drives to Speaker, waits, drives to Source.

### Phase 3: Autonomous Mode
1.  Disable Robot.
2.  On Shuffleboard "Auto" tab, select **"PathPlanner: TestAuto1"**.
3.  Enable **Autonomous**.
4.  *Expectation:* Robot follows the path defined in PathPlanner at a limited speed (0.5 m/s).
5.  *Collision Check:* If path hits a wall, robot will push safely (limited current) until timeout.

---

## 5. Troubleshooting (Logs)
If robot stops unexpectedly:
1.  Check Driver Station for **"Brownout Protection Active"** (Battery Issue).
2.  Check Shuffleboard Logs or Console for:
    *   `[VisionAssisted] Global timeout reached` -> Too slow / path too long.
    *   `[VisionAssisted] Vision phase timeout` -> Saw path end, but lost tag visibility.
