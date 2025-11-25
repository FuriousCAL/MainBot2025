# Pose Reset Test Utility - Removal Guide

This document explains how to remove the pose reset test utility after testing is complete.

## What Was Added

1. **New File**: `src/main/java/frc/robot/utils/PoseResetTestUtil.java`
   - Contains the pose reset utility class
   - Marked with `@Deprecated` annotation

2. **Modified File**: `src/main/java/frc/robot/RobotContainer.java`
   - Added import: `import frc.robot.utils.PoseResetTestUtil;`
   - Added field: `private PoseResetTestUtil poseResetUtil;` (around line 74)
   - Added method call: `configurePoseResetTestUtil();` in constructor (around line 147)
   - Added method: `configurePoseResetTestUtil()` (around line 373)
   - Added method: `getPoseResetUtil()` (around line 387)

## How to Use

1. Open Shuffleboard or Glass dashboard
2. Find the "TEST: Reset Robot Pose" dropdown chooser
3. Select a position (e.g., "Home Position", "Field Center", etc.)
4. Click the "TEST: Reset Pose to Selected" button
5. The robot's pose will be reset to the selected position

## How to Remove

### Step 1: Delete the utility file
```bash
rm src/main/java/frc/robot/utils/PoseResetTestUtil.java
```

### Step 2: Remove from RobotContainer.java

1. **Remove the import** (around line 31):
   ```java
   import frc.robot.utils.PoseResetTestUtil;  // DELETE THIS LINE
   ```

2. **Remove the field declaration** (around line 74):
   ```java
   // ============================================================================
   // TEST UTILITY: Pose Reset (REMOVE AFTER TESTING)
   // ============================================================================
   private PoseResetTestUtil poseResetUtil;  // DELETE THIS LINE
   // ============================================================================
   ```

3. **Remove the method call** (around line 147):
   ```java
   // ============================================================================
   // TEST UTILITY: Configure pose reset utility (REMOVE AFTER TESTING)
   // ============================================================================
   configurePoseResetTestUtil();  // DELETE THIS LINE
   // ============================================================================
   ```

4. **Remove the methods** (around lines 364-395):
   ```java
   // ============================================================================
   // TEST UTILITY: Pose Reset Configuration (REMOVE AFTER TESTING)
   // ============================================================================
   // DELETE EVERYTHING FROM HERE...
   private void configurePoseResetTestUtil() { ... }
   public PoseResetTestUtil getPoseResetUtil() { ... }
   // ...TO HERE
   // ============================================================================
   ```

### Step 3: Verify
- Compile the code to ensure no errors
- Check that the dashboard no longer shows the pose reset widgets

## Alternative: Quick Disable

If you want to keep the code but disable it temporarily, you can comment out the method call in the constructor:

```java
// configurePoseResetTestUtil();  // Temporarily disabled
```

This will prevent the widgets from appearing on the dashboard while keeping the code for future use.

