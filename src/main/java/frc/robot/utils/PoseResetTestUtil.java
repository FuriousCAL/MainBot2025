package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.AprilTagConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * TEST UTILITY: Pose Reset for Testing
 * 
 * This utility provides a dashboard interface to reset the robot's pose on the field
 * during testing. This is useful for testing autonomous routines from different starting
 * positions.
 * 
 * TO REMOVE: Delete this file and remove the call to configurePoseReset() in RobotContainer.
 * 
 * @deprecated This is a temporary testing utility and should be removed after testing.
 */
@Deprecated
public class PoseResetTestUtil {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final SendableChooser<Pose2d> poseChooser;
    
    // Predefined test positions
    private static final Pose2d HOME_POSITION = AprilTagConstants.HOME_POSITION; // (3.0, 3.0, 0°)
    private static final Pose2d FIELD_CENTER = AprilTagConstants.FIELD_CENTER;
    private static final Pose2d ORIGIN = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    private static final Pose2d BLUE_START_LEFT = new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(0.0));
    private static final Pose2d BLUE_START_CENTER = new Pose2d(1.5, 4.5, Rotation2d.fromDegrees(0.0));
    private static final Pose2d BLUE_START_RIGHT = new Pose2d(1.5, 3.5, Rotation2d.fromDegrees(0.0));
    private static final Pose2d RED_START_LEFT = new Pose2d(14.5, 5.5, Rotation2d.fromDegrees(180.0));
    private static final Pose2d RED_START_CENTER = new Pose2d(14.5, 4.5, Rotation2d.fromDegrees(180.0));
    private static final Pose2d RED_START_RIGHT = new Pose2d(14.5, 3.5, Rotation2d.fromDegrees(180.0));
    
    /**
     * Creates a new PoseResetTestUtil.
     * 
     * @param drivetrain The drivetrain subsystem to reset
     */
    public PoseResetTestUtil(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.poseChooser = new SendableChooser<>();
        setupChooser();
    }
    
    /**
     * Sets up the pose chooser with predefined positions.
     */
    private void setupChooser() {
        poseChooser.setDefaultOption("Home Position (3.0, 3.0, 0°)", HOME_POSITION);
        poseChooser.addOption("Field Center", FIELD_CENTER);
        poseChooser.addOption("Origin (0, 0, 0°)", ORIGIN);
        poseChooser.addOption("Blue Start Left", BLUE_START_LEFT);
        poseChooser.addOption("Blue Start Center", BLUE_START_CENTER);
        poseChooser.addOption("Blue Start Right", BLUE_START_RIGHT);
        poseChooser.addOption("Red Start Left", RED_START_LEFT);
        poseChooser.addOption("Red Start Center", RED_START_CENTER);
        poseChooser.addOption("Red Start Right", RED_START_RIGHT);
        
        // Add to SmartDashboard
        SmartDashboard.putData("TEST: Reset Robot Pose", poseChooser);
    }
    
    /**
     * Resets the robot's pose to the currently selected position in the chooser.
     * Call this periodically (e.g., in robotPeriodic) to check for pose reset requests.
     * 
     * This method checks if a new selection was made and resets the pose accordingly.
     * Note: This is a simple implementation. For more robust handling, you might want
     * to use a button widget or check for changes in selection.
     */
    public void update() {
        // This method can be called periodically to check for pose reset requests
        // For now, the chooser is available on the dashboard for manual selection
        // You can add button-based reset logic here if needed
    }
    
    /**
     * Resets the robot's pose to the specified position.
     * 
     * @param pose The pose to reset to
     */
    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
        System.out.println(String.format(
            "[PoseResetTest] Robot pose reset to: (%.2f, %.2f, %.1f°)",
            pose.getX(), pose.getY(), pose.getRotation().getDegrees()
        ));
    }
    
    /**
     * Resets the robot's pose to the currently selected position in the chooser.
     */
    public void resetToSelectedPose() {
        Pose2d selectedPose = poseChooser.getSelected();
        if (selectedPose != null) {
            resetPose(selectedPose);
        }
    }
    
    /**
     * Gets the currently selected pose from the chooser.
     * 
     * @return The selected pose, or null if none selected
     */
    public Pose2d getSelectedPose() {
        return poseChooser.getSelected();
    }
}

