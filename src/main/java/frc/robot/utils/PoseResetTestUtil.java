package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.constants.AprilTagConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * TEST UTILITY: Pose Reset for Testing
 * 
 * This utility provides a dashboard interface (Shuffleboard Tab) to reset the robot's pose 
 * on the field during testing.
 */
public class PoseResetTestUtil {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final SendableChooser<Pose2d> poseChooser;
    
    // Predefined test positions
    private static final Pose2d HOME_POSITION = AprilTagConstants.HOME_POSITION;
    private static final Pose2d FIELD_CENTER = AprilTagConstants.FIELD_CENTER;
    private static final Pose2d ORIGIN = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    private static final Pose2d BLUE_START_LEFT = new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(0.0));
    private static final Pose2d BLUE_START_CENTER = new Pose2d(1.5, 4.5, Rotation2d.fromDegrees(0.0));
    private static final Pose2d BLUE_START_RIGHT = new Pose2d(1.5, 3.5, Rotation2d.fromDegrees(0.0));
    private static final Pose2d RED_START_LEFT = new Pose2d(14.5, 5.5, Rotation2d.fromDegrees(180.0));
    private static final Pose2d RED_START_CENTER = new Pose2d(14.5, 4.5, Rotation2d.fromDegrees(180.0));
    private static final Pose2d RED_START_RIGHT = new Pose2d(14.5, 3.5, Rotation2d.fromDegrees(180.0));
    
    public PoseResetTestUtil(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.poseChooser = new SendableChooser<>();
        setupChooser();
        setupDashboard();
    }
    
    private void setupChooser() {
        poseChooser.setDefaultOption("Home Position (3.0, 3.0, 0°)", HOME_POSITION);
        poseChooser.addOption("Field Center", FIELD_CENTER);
        poseChooser.addOption("Origin (0, 0, 0°)", ORIGIN);
        
        poseChooser.addOption("Blue Start Left (Top)", BLUE_START_LEFT);
        poseChooser.addOption("Blue Start Center", BLUE_START_CENTER);
        poseChooser.addOption("Blue Start Right (Bottom)", BLUE_START_RIGHT);
        
        poseChooser.addOption("Red Start Left (Top)", RED_START_LEFT);
        poseChooser.addOption("Red Start Center", RED_START_CENTER);
        poseChooser.addOption("Red Start Right (Bottom)", RED_START_RIGHT);
    }
    
    private void setupDashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Match Setup");
        
        tab.add("Start Position", poseChooser)
           .withPosition(0, 0)
           .withSize(2, 1);
           
        tab.add("RESET POSE", new InstantCommand(this::resetToSelectedPose))
           .withPosition(2, 0)
           .withSize(2, 1);
    }
    
    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
        System.out.println(String.format(
            "[PoseResetTest] Robot pose reset to: (%.2f, %.2f, %.1f°)",
            pose.getX(), pose.getY(), pose.getRotation().getDegrees()
        ));
    }
    
    public void resetToSelectedPose() {
        Pose2d selectedPose = poseChooser.getSelected();
        if (selectedPose != null) {
            resetPose(selectedPose);
        }
    }
}
