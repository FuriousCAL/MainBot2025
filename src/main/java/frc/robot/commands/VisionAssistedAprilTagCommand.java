package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AprilTagConstants;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.PathPlannerUtils;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Vision-Assisted AprilTag Navigation Command
 * 
 * This command implements a two-phase approach:
 * 1. Coarse Navigation: Uses PathPlanner to get within ~2.0m of target AprilTag
 * 2. Precision Control: Uses real-time vision feedback for exact positioning
 * 
 * Final position: 0.25m directly in front of the specified AprilTag
 */
public class VisionAssistedAprilTagCommand extends Command {
    
    // ==========================================================================
    // CONSTANTS
    // ==========================================================================
    
    /** Distance to maintain from AprilTag center (meters) */
    private static final double TARGET_DISTANCE_METERS = 0.25;
    
    /** Distance threshold to switch from PathPlanner to vision control */
    private static final double VISION_SWITCH_DISTANCE = 2.0;
    
    /** Maximum pose ambiguity to accept for vision control */
    private static final double MAX_VISION_AMBIGUITY = 0.3;
    
    /** Timeout for PathPlanner phase (seconds) - Base value, scaled by Safety Mode */
    private static final double PATHPLANNER_TIMEOUT_BASE = 10.0;
    
    /** Timeout for vision precision phase (seconds) - Base value, scaled by Safety Mode */
    private static final double VISION_TIMEOUT_BASE = 5.0;
    
    /** Position tolerance for completion (meters) */
    private static final double POSITION_TOLERANCE = 0.02;
    
    /** Angle tolerance for completion (degrees) */
    private static final double ANGLE_TOLERANCE = 1.0;
    
    // ==========================================================================
    // SUBSYSTEMS AND CONTROLLERS
    // ==========================================================================
    
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final int targetTagId;
    
    /** Holonomic drive controller for vision-based precision control */
    private final HolonomicDriveController visionController;
    
    /** PathPlanner command for coarse navigation */
    private Command pathPlannerCommand;
    
    // ==========================================================================
    // STATE VARIABLES
    // ==========================================================================
    
    /** Current command phase */
    private enum Phase { PATHPLANNER, VISION, FINISHED }
    private Phase currentPhase = Phase.PATHPLANNER;
    
    // Telemetry throttling
    private int telemetryCounter = 0;

    /** Command start time for timeout handling */
    private double commandStartTime;
    private double phaseStartTime;
    
    /** Target pose from field layout (fallback) */
    private Pose2d targetPose;
    
    /** Final target pose (0.25m in front of tag, facing tag) */
    private Pose2d finalTargetPose;
    
    /**
     * Creates a new VisionAssistedAprilTagCommand.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param visionSubsystem The vision subsystem
     * @param tagId The AprilTag ID to navigate to
     */
    public VisionAssistedAprilTagCommand(CommandSwerveDrivetrain drivetrain, 
                                       VisionSubsystem visionSubsystem, 
                                       int tagId) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.targetTagId = tagId;
        
        // Get target pose from field layout
        Optional<Pose2d> tagPoseOpt = AprilTagConstants.getTagPose(tagId);
        this.targetPose = tagPoseOpt.orElse(AprilTagConstants.FIELD_CENTER);
        
        // Calculate final target pose (0.25m in front of tag, facing tag)
        this.finalTargetPose = calculateFinalTargetPose(targetPose);
        
        // Create vision controller with appropriate PID gains
        // These gains are for the final precision approach
        this.visionController = new HolonomicDriveController(
            new PIDController(2.5, 0.0, 0.0),  // X controller
            new PIDController(2.5, 0.0, 0.0),  // Y controller
            new ProfiledPIDController(3.0, 0.0, 0.0,  // Theta controller
                new TrapezoidProfile.Constraints(4.0, 3.0))  // Max vel: 4 rad/s, Max accel: 3 rad/s²
        );
        
        // Set position and angle tolerances
        visionController.setTolerance(
            new Pose2d(POSITION_TOLERANCE, POSITION_TOLERANCE, 
                      Rotation2d.fromDegrees(ANGLE_TOLERANCE))
        );
        
        addRequirements(drivetrain, visionSubsystem);
    }
    
    // ==========================================================================
    // COMMAND LIFECYCLE
    // ==========================================================================
    
    @Override
    public void initialize() {
        commandStartTime = Timer.getFPGATimestamp();
        phaseStartTime = commandStartTime;
        currentPhase = Phase.PATHPLANNER;
        
        // Refresh target pose in case field layout changed
        AprilTagConstants.getTagPose(targetTagId).ifPresent(pose -> {
            this.targetPose = pose;
            this.finalTargetPose = calculateFinalTargetPose(pose);
        });
        
        // Calculate an approach position slightly further back for pathfinding
        // This prevents PathPlanner from trying to drive *through* the tag if starting nearby
        Pose2d approachPose = calculateApproachPose(targetPose, 1.0); // 1.0m standoff
        
        // Create PathPlanner command for initial approach
        // We use the approach pose to ensure we arrive facing the tag
        pathPlannerCommand = AutoBuilder.pathfindToPose(
            approachPose,
            PathPlannerUtils.getDefaultPathConstraints()
        );
        
        pathPlannerCommand.initialize();
        
        SmartDashboard.putString("VisionAssisted/Phase", "PATHPLANNER");
        SmartDashboard.putNumber("VisionAssisted/Target Tag", targetTagId);
        SmartDashboard.putString("VisionAssisted/Status", "Starting PathPlanner approach");
        
        System.out.println(String.format(
            "[VisionAssisted] Starting approach to Tag %d. Final Target: (%.2f, %.2f)",
            targetTagId, finalTargetPose.getX(), finalTargetPose.getY()
        ));
    }
    
    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        double elapsedTime = currentTime - commandStartTime;
        double phaseElapsedTime = currentTime - phaseStartTime;
        
        // Update telemetry every 10 loops (5Hz) to prevent loop overruns
        boolean updateTelemetry = (telemetryCounter++ % 10 == 0);
        
        if (updateTelemetry) {
            SmartDashboard.putNumber("VisionAssisted/Elapsed Time", elapsedTime);
            SmartDashboard.putNumber("VisionAssisted/Phase Time", phaseElapsedTime);
        }
        
        switch (currentPhase) {
            case PATHPLANNER:
                executePathPlannerPhase(phaseElapsedTime, updateTelemetry);
                break;
            case VISION:
                executeVisionPhase(phaseElapsedTime, updateTelemetry);
                break;
            case FINISHED:
                drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
                break;
        }
    }
    
    @Override
    public boolean isFinished() {
        // Safety timeout (Extended for SAFETY_TESTING_MODE)
        double safetyMultiplier = Constants.Safety.SAFETY_TESTING_MODE ? 5.0 : 1.0;
        double globalTimeout = (PATHPLANNER_TIMEOUT_BASE + VISION_TIMEOUT_BASE + 5.0) * safetyMultiplier;
        
        if (Timer.getFPGATimestamp() - commandStartTime > globalTimeout) {
            DataLogManager.log("[VisionAssisted] Global timeout reached (Limit: " + globalTimeout + "s)");
            System.out.println("[VisionAssisted] Global timeout reached");
            return true;
        }
        return currentPhase == Phase.FINISHED;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop drivetrain
        drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        
        // Clean up PathPlanner command
        if (pathPlannerCommand != null) {
            pathPlannerCommand.end(interrupted);
        }
        
        String status = interrupted ? "INTERRUPTED" : 
                       (currentPhase == Phase.FINISHED ? "COMPLETED" : "TIMEOUT");
        
        SmartDashboard.putString("VisionAssisted/Status", status);
        
        System.out.println(String.format(
            "[VisionAssisted] Command ended: %s (Phase: %s, Distance: %.3fm)",
            status, currentPhase, getDistanceToTarget()
        ));
    }
    
    // ==========================================================================
    // PHASE EXECUTION METHODS
    // ==========================================================================
    
    /**
     * Executes the PathPlanner coarse navigation phase.
     */
    private void executePathPlannerPhase(double phaseElapsedTime, boolean updateTelemetry) {
        // Continue PathPlanner command
        if (pathPlannerCommand != null) {
            pathPlannerCommand.execute();
        }
        
        // Calculate timeout based on mode
        double timeout = PATHPLANNER_TIMEOUT_BASE * (Constants.Safety.SAFETY_TESTING_MODE ? 5.0 : 1.0);
        
        // Check conditions to switch to vision control
        double distanceToTarget = getDistanceToTarget();
        boolean closeEnough = distanceToTarget < VISION_SWITCH_DISTANCE;
        boolean pathPlannerFinished = pathPlannerCommand != null && pathPlannerCommand.isFinished();
        boolean pathPlannerTimeout = phaseElapsedTime > timeout;
        
        if (updateTelemetry) {
            SmartDashboard.putNumber("VisionAssisted/Distance to Target", distanceToTarget);
            SmartDashboard.putBoolean("VisionAssisted/PathPlanner Finished", pathPlannerFinished);
        }
        
        // Switch if we are close enough OR PathPlanner finished/timed out
        if (closeEnough || pathPlannerFinished || pathPlannerTimeout) {
            if (pathPlannerTimeout) {
                DataLogManager.log("[VisionAssisted] PathPlanner phase timed out (" + timeout + "s), switching to Vision anyway");
            } else if (closeEnough) { 
                DataLogManager.log("[VisionAssisted] Close enough for Vision (" + distanceToTarget + "m)");
            }
            switchToVisionPhase();
        }
    }
    
    /**
     * Executes the vision-based precision control phase.
     */
    private void executeVisionPhase(double phaseElapsedTime, boolean updateTelemetry) {
        // Check for vision timeout
        double timeout = VISION_TIMEOUT_BASE * (Constants.Safety.SAFETY_TESTING_MODE ? 5.0 : 1.0);
        if (phaseElapsedTime > timeout) {
            currentPhase = Phase.FINISHED;
            DataLogManager.log("[VisionAssisted] Vision phase timed out after " + timeout + "s. Final Dist: " + getDistanceToTarget());
            SmartDashboard.putString("VisionAssisted/Status", "Vision phase timeout");
            return;
        }
        
        // Get current robot pose
        Pose2d currentPose = drivetrain.getPose();
        
        // Update target pose from vision if available (optional refinement)
        // For now, we trust the AprilTagConstants map and the robot's fused odometry
        
        // Calculate chassis speeds using holonomic controller
        ChassisSpeeds chassisSpeeds = visionController.calculate(
            currentPose,
            finalTargetPose,
            0.0,  // No desired velocity at target
            finalTargetPose.getRotation()
        );
        
        // Distance-based speed scaling for smooth arrival
        double distance = currentPose.getTranslation().getDistance(finalTargetPose.getTranslation());
        
        // Use direct vision measurement if available for better close-range accuracy
        if (visionSubsystem.isTagVisible(targetTagId)) {
            var targetOpt = visionSubsystem.getBestTarget();
            if (targetOpt.isPresent()) {
                var target = targetOpt.get();
                if (target.getFiducialId() == targetTagId) {
                    // Vision gives us distance to the tag itself. 
                    // Our target is TARGET_DISTANCE_METERS in front of it.
                    double distanceToTag = target.getBestCameraToTarget().getTranslation().getNorm();
                    double visionDistance = Math.max(0.0, distanceToTag - TARGET_DISTANCE_METERS);
                    
                    // Trust vision more when we are close (within 1.5m)
                    if (distanceToTag < 1.5) {
                        distance = visionDistance;
                    }
                }
            }
        }

        // Drastic slowing profile (Quadratic)
        // Start slowing at 2.0m (start of vision phase)
        double slowDownRadius = 2.0;
        double minSpeed = 0.15; // Minimum speed to overcome friction
        double speedScale = 1.0;
        
        if (distance < slowDownRadius) {
            // Quadratic curve: Drops off faster than linear
            // 2.0m -> 100%
            // 1.0m -> 25%
            // 0.5m -> 6% (clamped to min)
            speedScale = Math.pow(distance / slowDownRadius, 2);
        }
        
        speedScale = Math.max(minSpeed, speedScale);
        
        // Limit maximum speeds for safety during precision control
        // Close range: slow down significantly
        double maxLinearSpeed = 1.0 * speedScale;
        double maxAngularSpeed = 1.5; // rad/s
        
        chassisSpeeds.vxMetersPerSecond = MathUtil.clamp(chassisSpeeds.vxMetersPerSecond, -maxLinearSpeed, maxLinearSpeed);
        chassisSpeeds.vyMetersPerSecond = MathUtil.clamp(chassisSpeeds.vyMetersPerSecond, -maxLinearSpeed, maxLinearSpeed);
        chassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(chassisSpeeds.omegaRadiansPerSecond, -maxAngularSpeed, maxAngularSpeed);
        
        // Apply chassis speeds (robot relative)
        drivetrain.driveRobotRelative(chassisSpeeds);
        
        // Update telemetry
        if (updateTelemetry) {
            SmartDashboard.putNumber("VisionAssisted/Target X", finalTargetPose.getX());
            SmartDashboard.putNumber("VisionAssisted/Target Y", finalTargetPose.getY());
            SmartDashboard.putNumber("VisionAssisted/Target Rotation", finalTargetPose.getRotation().getDegrees());
            SmartDashboard.putNumber("VisionAssisted/Current Distance", distance);
        }
        
        // Check if we're at the target
        if (visionController.atReference()) {
            currentPhase = Phase.FINISHED;
            SmartDashboard.putString("VisionAssisted/Status", "Target reached!");
            System.out.println("[VisionAssisted] Successfully reached target position");
        }
    }
    
    // ==========================================================================
    // HELPER METHODS
    // ==========================================================================
    
    /**
     * Switches from PathPlanner phase to vision precision phase.
     */
    private void switchToVisionPhase() {
        // End PathPlanner command if running
        if (pathPlannerCommand != null) {
            pathPlannerCommand.end(false);
        }
        
        currentPhase = Phase.VISION;
        phaseStartTime = Timer.getFPGATimestamp();
        
        SmartDashboard.putString("VisionAssisted/Phase", "VISION");
        SmartDashboard.putString("VisionAssisted/Status", "Switching to vision precision control");
        
        // Reset PID controllers for smooth transition
        // Reset theta controller to current heading
        visionController.getThetaController().reset(drivetrain.getPose().getRotation().getRadians());
        
        System.out.println("[VisionAssisted] Switching to vision precision control");
    }
    
    /**
     * Calculates the final target pose (TARGET_DISTANCE_METERS in front of tag, facing tag).
     */
    private Pose2d calculateFinalTargetPose(Pose2d tagPose) {
        // Calculate position TARGET_DISTANCE_METERS in front of the tag
        // Tags face OUT from the wall. We want to be in front of it.
        // The tag's X axis points OUT from the tag.
        
        // Vector pointing out from tag: (TARGET_DISTANCE_METERS, 0) relative to tag
        Translation2d vectorOutFromTag = new Translation2d(TARGET_DISTANCE_METERS, 0.0);
        
        // Rotate this vector by the tag's rotation to get field-relative offset
        Translation2d fieldRelativeOffset = vectorOutFromTag.rotateBy(tagPose.getRotation());
        
        // Target position is tag position + vector out
        Translation2d targetTranslation = tagPose.getTranslation().plus(fieldRelativeOffset);
        
        // We want to face the tag.
        // Tag faces OUT. We want to face IN (opposing the tag).
        // Tag Rotation + 180 degrees.
        Rotation2d targetRotation = tagPose.getRotation().plus(Rotation2d.fromDegrees(180.0));
        
        return new Pose2d(targetTranslation, targetRotation);
    }
    
    /**
     * Calculates an approach pose for PathPlanner (further back than final target).
     */
    private Pose2d calculateApproachPose(Pose2d tagPose, double distanceMeters) {
        // Same logic as above, but further away
        Translation2d vectorOutFromTag = new Translation2d(distanceMeters, 0.0);
        Translation2d fieldRelativeOffset = vectorOutFromTag.rotateBy(tagPose.getRotation());
        Translation2d targetTranslation = tagPose.getTranslation().plus(fieldRelativeOffset);
        Rotation2d targetRotation = tagPose.getRotation().plus(Rotation2d.fromDegrees(180.0));
        
        return new Pose2d(targetTranslation, targetRotation);
    }
    
    /**
     * Updates the final target pose based on live vision data.
     * This helps correct for field map inaccuracies or tag shifts.
     */
    private void updateTargetPoseFromVision() {
        if (!visionSubsystem.isTagVisible(targetTagId)) {
            return;
        }
        
        // Only update if we have a good target
        visionSubsystem.getBestTarget().ifPresent(target -> {
            if (target.getFiducialId() != targetTagId) return;
            if (target.getPoseAmbiguity() > MAX_VISION_AMBIGUITY) return;
            
            // Note: We primarily rely on the fused robot pose which VisionSubsystem updates.
            // But we could add advanced visual servoing logic here if needed.
        });
    }
    
    /**
     * Gets the distance from robot to target position.
     */
    private double getDistanceToTarget() {
        Pose2d currentPose = drivetrain.getPose();
        return currentPose.getTranslation().getDistance(finalTargetPose.getTranslation());
    }
    
    /**
     * Checks if we have a good vision target for the specified AprilTag.
     */
    private boolean hasGoodVisionTarget() {
        if (!visionSubsystem.isTagVisible(targetTagId)) {
            return false;
        }
        
        Optional<PhotonTrackedTarget> bestTarget = visionSubsystem.getBestTarget();
        if (bestTarget.isEmpty()) {
            return false;
        }
        
        PhotonTrackedTarget target = bestTarget.get();
        return target.getFiducialId() == targetTagId && 
               target.getPoseAmbiguity() < MAX_VISION_AMBIGUITY;
    }
}
