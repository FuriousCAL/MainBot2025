package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.constants.AprilTagConstants;

public class DriveToAprilTag2Command extends Command {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    
    private final PIDController xController = new PIDController(2.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(2.0, 0.0, 0.0);
    private final PIDController rotationController = new PIDController(3.0, 0.0, 0.0);
    
    private final Pose2d targetPose;
    
    // FIXED: Increased rotation tolerance from 10 to 30
    private static final double POSITION_TOLERANCE = 0.5; // meters
    private static final double ROTATION_TOLERANCE = Math.toRadians(30); // degrees
    
    // NEW: Added timeout mechanism
    private static final double TIMEOUT = 10.0; // seconds
    private double startTime;
    
    public DriveToAprilTag2Command(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        
        this.targetPose = AprilTagConstants.getTagPose(2).orElse(
            new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(0))
        );
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        System.out.println("DriveToAprilTag2Command: Starting to drive toward AprilTag 2");
        System.out.println("Target position: " + targetPose);
        
        xController.reset();
        yController.reset();
        rotationController.reset();
        
        startTime = Timer.getFPGATimestamp();
    }
    
    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        double rotationError = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();
        
        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double rotationSpeed = rotationController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        
        xSpeed = Math.max(-1.0, Math.min(1.0, xSpeed));
        ySpeed = Math.max(-1.0, Math.min(1.0, ySpeed));
        rotationSpeed = Math.max(-1.0, Math.min(1.0, rotationSpeed));
        
        // Convert field-relative PID outputs to robot-relative speeds
        var fieldRelativeSpeeds = new edu.wpi.first.math.kinematics.ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
        var robotRelativeSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds, 
            currentPose.getRotation()
        );
        
        drivetrain.driveRobotRelative(robotRelativeSpeeds);
        
        // Log errors to SmartDashboard (NetworkTables -> DataLog)
        SmartDashboard.putNumber("DriveToAprilTag2/X Error", xError);
        SmartDashboard.putNumber("DriveToAprilTag2/Y Error", yError);
        SmartDashboard.putNumber("DriveToAprilTag2/Rotation Error Deg", Math.toDegrees(rotationError));
        SmartDashboard.putNumber("DriveToAprilTag2/Time", Timer.getFPGATimestamp() - startTime);
    }
    
    @Override
    public boolean isFinished() {
        Pose2d currentPose = drivetrain.getPose();
        
        double xError = Math.abs(targetPose.getX() - currentPose.getX());
        double yError = Math.abs(targetPose.getY() - currentPose.getY());
        double rotationError = Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getRadians());
        
        boolean atPosition = xError < POSITION_TOLERANCE && yError < POSITION_TOLERANCE;
        boolean atRotation = rotationError < ROTATION_TOLERANCE;
        boolean timeoutReached = (Timer.getFPGATimestamp() - startTime) > TIMEOUT;
        
        if (atPosition && atRotation) {
            System.out.println("DriveToAprilTag2Command: Arrived at AprilTag 2!");
            return true;
        }
        
        if (timeoutReached) {
            System.out.println("DriveToAprilTag2Command: Timeout reached, stopping");
            return true;
        }
        
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.driveRobotRelative(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0));
        
        if (interrupted) {
            System.out.println("DriveToAprilTag2Command: Interrupted");
        } else {
            System.out.println("DriveToAprilTag2Command: Completed successfully");
        }
    }
}
