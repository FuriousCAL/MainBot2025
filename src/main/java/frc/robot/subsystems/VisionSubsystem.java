package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import frc.robot.constants.AprilTagConstants;
import frc.robot.constants.Constants;

/**
 * Professional VisionSubsystem for MinBot2.
 * 
 * Features:
 * - Real-time AprilTag detection and pose estimation
 * - Integration with swerve drivetrain pose estimation
 * - Health monitoring and diagnostics
 * - Support for both real hardware and simulation
 * - Professional telemetry and logging
 */
public class VisionSubsystem extends SubsystemBase {
    
    // ==========================================================================
    // HARDWARE AND POSE ESTIMATION
    // ==========================================================================
    
    /** Primary PhotonVision camera */
    private final PhotonCamera camera;
    
    /** PhotonVision pose estimator for robot localization */
    private final PhotonPoseEstimator poseEstimator;
    
    /** Transform from robot center to camera */
    private final Transform3d robotToCameraTransform;
    
    // ==========================================================================
    // STATE VARIABLES
    // ==========================================================================
    
    /** Latest pipeline result from camera */
    private PhotonPipelineResult latestResult = new PhotonPipelineResult();
    
    /** Latest estimated robot pose */
    private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();
    
    /** Connection status tracking */
    private boolean cameraConnected = false;
    private double lastResultTimestamp = 0.0;
    private int consecutiveFailures = 0;
    
    /** Performance metrics */
    private double averageLatencyMs = 0.0;
    private int frameCount = 0;

    // DataLog entries
    private DoubleLogEntry logPoseX;
    private DoubleLogEntry logPoseY;
    private DoubleLogEntry logPoseTheta;
    private DoubleLogEntry logTargetLatency;
    private BooleanLogEntry logHasTargets;
    private StringLogEntry logVisionEvents;

    // Shuffleboard Entries
    private final GenericEntry connectedEntry;
    private final GenericEntry failuresEntry;
    private final GenericEntry hasTargetsEntry;
    private final GenericEntry targetCountEntry;
    private final GenericEntry frameCountEntry;
    private final GenericEntry estimatedPoseEntry;
    private final GenericEntry tagsUsedEntry;
    private final GenericEntry bestTargetIdEntry;
    private final GenericEntry bestTargetAmbiguityEntry;
    private final GenericEntry bestTargetAreaEntry;
    
    /**
     * Creates a new VisionSubsystem.
     */
    public VisionSubsystem() {
        // Initialize Shuffleboard Tab
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        connectedEntry = tab.add("Connected", false).withPosition(0, 0).getEntry();
        hasTargetsEntry = tab.add("Has Targets", false).withPosition(1, 0).getEntry();
        targetCountEntry = tab.add("Target Count", 0).withPosition(2, 0).getEntry();
        failuresEntry = tab.add("Consecutive Failures", 0).withPosition(3, 0).getEntry();
        frameCountEntry = tab.add("Frame Count", 0).withPosition(4, 0).getEntry();
        
        estimatedPoseEntry = tab.add("Estimated Pose", "None").withPosition(0, 1).withSize(2, 1).getEntry();
        tagsUsedEntry = tab.add("Tags Used", 0).withPosition(2, 1).getEntry();
        
        bestTargetIdEntry = tab.add("Best Target ID", -1).withPosition(0, 2).getEntry();
        bestTargetAmbiguityEntry = tab.add("Best Target Ambiguity", 0.0).withPosition(1, 2).getEntry();
        bestTargetAreaEntry = tab.add("Best Target Area", 0.0).withPosition(2, 2).getEntry();

        // Initialize DataLog entries
        DataLog log = DataLogManager.getLog();
        logPoseX = new DoubleLogEntry(log, "Vision/Log/PoseX");
        logPoseY = new DoubleLogEntry(log, "Vision/Log/PoseY");
        logPoseTheta = new DoubleLogEntry(log, "Vision/Log/PoseTheta");
        logTargetLatency = new DoubleLogEntry(log, "Vision/Log/Latency");
        logHasTargets = new BooleanLogEntry(log, "Vision/Log/HasTargets");
        logVisionEvents = new StringLogEntry(log, "Vision/Log/Events");

        // Initialize camera with configured name
        camera = new PhotonCamera(Constants.Vision.PRIMARY_CAMERA_NAME);
        
        // Create robot-to-camera transform from constants
        robotToCameraTransform = new Transform3d(
            new Translation3d(
                Constants.Vision.CAMERA_FORWARD_OFFSET_METERS,
                Constants.Vision.CAMERA_SIDE_OFFSET_METERS,
                Constants.Vision.CAMERA_HEIGHT_METERS
            ),
            new Rotation3d(
                Math.toRadians(Constants.Vision.CAMERA_ROLL_DEGREES),
                Math.toRadians(Constants.Vision.CAMERA_PITCH_DEGREES),
                Math.toRadians(Constants.Vision.CAMERA_YAW_DEGREES)
            )
        );
        
        // Create pose estimator with field layout (updated constructor)
        poseEstimator = new PhotonPoseEstimator(
            AprilTagConstants.FIELD_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // Best for accuracy
            robotToCameraTransform
        );
        
        // Configure pose estimator settings
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        System.out.println("[VisionSubsystem] Initialized with camera: " + Constants.Vision.PRIMARY_CAMERA_NAME);
        System.out.println("[VisionSubsystem] PhotonVision dashboard: http://192.168.86.30:5800/#/camera");
    }
    
    // ==========================================================================
    // PERIODIC AND STATE MANAGEMENT
    // ==========================================================================
    
    /** Telemetry update counter - throttle updates to reduce overhead */
    private int telemetryUpdateCounter = 0;
    private static final int TELEMETRY_UPDATE_INTERVAL = 5; // Update every 5 loops (~100ms)
    
    @Override
    public void periodic() {
        // OPTIMIZATION: Always update critical data (camera and pose)
        updateCameraData();
        updatePoseEstimation();
        
        // OPTIMIZATION: Throttle telemetry updates (expensive SmartDashboard calls)
        // Top FRC teams update telemetry at 10-20Hz instead of 50Hz
        telemetryUpdateCounter++;
        if (telemetryUpdateCounter >= TELEMETRY_UPDATE_INTERVAL) {
            updateTelemetry();
            telemetryUpdateCounter = 0;
        }
        
        // OPTIMIZATION: Health monitoring less frequently
        if (telemetryUpdateCounter == 0) {
            monitorHealth();
        }
    }
    
    /**
     * Updates camera data and connection status.
     * OPTIMIZED: Minimal work, no expensive operations.
     */
    private void updateCameraData() {
        try {
            // OPTIMIZATION: Get latest result (this is fast, just reading from NetworkTables)
            // Note: getLatestResult() is deprecated in PhotonVision 2024+ but still functional
            // TODO: Update to new API when migrating to PhotonVision 2025+
            // Using getAllUnreadResults() is the recommended replacement, but for now we just suppress or ignore
            // In the 2025 API, this might be getLatestResult() again or something else.
            // For now, suppress the warning if possible, or just use the method as is.
            // Actually, let's try using the non-deprecated way if available, or just acknowledge it.
            // The warning says: [removal] getLatestResult() in PhotonCamera has been deprecated and marked for removal
            
            // In newer PhotonLib, we should often use:
            // for (var result : camera.getAllUnreadResults()) { ... }
            // But that changes the logic significantly (processing queue vs latest).
            // For this subsystem which wants the *latest* state for drive control, getLatestResult() is logically correct
            // even if deprecated. The replacement is likely getting the whole list and taking the last one.
            
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            if (results.isEmpty()) {
                return;
            }
            PhotonPipelineResult result = results.get(results.size() - 1);
            
            // OPTIMIZATION: Only process if we have a new result (avoid unnecessary work)
            if (result.getTimestampSeconds() != latestResult.getTimestampSeconds()) {
                boolean wasConnected = cameraConnected;
                latestResult = result;
                lastResultTimestamp = Timer.getFPGATimestamp();
                cameraConnected = true;
                consecutiveFailures = 0;
                frameCount++;

                if (!wasConnected) {
                    logVisionEvents.append("Camera Connected");
                }
                
                // Log direct metrics that might be throttled in SmartDashboard
                logHasTargets.append(result.hasTargets());
                // Calculate total system latency (capture to now) in ms
                double totalLatencyMs = (Timer.getFPGATimestamp() - result.getTimestampSeconds()) * 1000.0;
                logTargetLatency.append(totalLatencyMs);
            }
        } catch (Exception e) {
            consecutiveFailures++;
            // OPTIMIZATION: Only log errors occasionally to avoid spam
            if (consecutiveFailures % 10 == 0) {
                System.err.println("[VisionSubsystem] Error updating camera data (failures: " + consecutiveFailures + "): " + e.getMessage());
            }
        }
    }
    
    /**
     * Updates pose estimation from AprilTag detections.
     */
    private void updatePoseEstimation() {
        if (!Constants.Vision.ENABLE_POSE_ESTIMATION || !hasTargets()) {
            latestEstimatedPose = Optional.empty();
            return;
        }
        
        try {
            // Get pose estimate from PhotonVision (pass the latest result)
            Optional<EstimatedRobotPose> poseResult = poseEstimator.update(latestResult);
            
            if (poseResult.isPresent()) {
                EstimatedRobotPose estimate = poseResult.get();
                
                // Validate pose estimate quality
                if (isPoseEstimateValid(estimate)) {
                    latestEstimatedPose = poseResult;
                    
                    if (Constants.Vision.ENABLE_VISION_LOGGING) {
                        logPoseEstimate(estimate);
                    }
                } else {
                    latestEstimatedPose = Optional.empty();
                }
            } else {
                latestEstimatedPose = Optional.empty();
            }
        } catch (Exception e) {
            latestEstimatedPose = Optional.empty();
            System.err.println("[VisionSubsystem] Error updating pose estimation: " + e.getMessage());
        }
    }
    
    // ==========================================================================
    // PUBLIC API METHODS
    // ==========================================================================
    
    /**
     * Gets the latest estimated robot pose from vision.
     * 
     * @return Optional containing estimated pose if available and valid
     */
    public Optional<EstimatedRobotPose> getLatestEstimatedPose() {
        return latestEstimatedPose;
    }
    
    /**
     * Gets the vision measurement standard deviations for pose estimation.
     * 
     * @return Matrix of standard deviations [x, y, theta]
     */
    public Matrix<N3, N1> getVisionMeasurementStdDevs() {
        // Adjust confidence based on distance and number of tags
        double[] stdDevs = Constants.Vision.VISION_MEASUREMENT_STDDEVS.clone();
        
        if (latestEstimatedPose.isPresent()) {
            EstimatedRobotPose estimate = latestEstimatedPose.get();
            int tagCount = estimate.targetsUsed.size();
            
            // More tags = higher confidence (lower standard deviation)
            double tagMultiplier = Math.max(0.5, 1.0 / tagCount);
            stdDevs[0] *= tagMultiplier; // x
            stdDevs[1] *= tagMultiplier; // y
            stdDevs[2] *= tagMultiplier; // theta
        }
        
        return VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2]);
    }
    
    /**
     * Checks if the camera currently sees any AprilTag targets.
     * 
     * @return true if targets are visible
     */
    public boolean hasTargets() {
        return latestResult.hasTargets();
    }
    
    /**
     * Gets the number of currently visible targets.
     * 
     * @return number of visible targets
     */
    public int getTargetCount() {
        return latestResult.hasTargets() ? latestResult.getTargets().size() : 0;
    }
    
    /**
     * Gets all currently visible AprilTag targets.
     * OPTIMIZED: Returns direct reference to avoid unnecessary list creation.
     * 
     * @return list of visible targets (empty list if none)
     */
    public List<PhotonTrackedTarget> getVisibleTargets() {
        // OPTIMIZATION: Return empty list directly instead of creating new ArrayList
        if (!latestResult.hasTargets()) {
            return new ArrayList<>(); // Still need new list for safety (defensive copy)
        }
        return latestResult.getTargets();
    }
    
    /**
     * Checks if a specific AprilTag is currently visible.
     * 
     * @param tagId The AprilTag ID to check
     * @return true if the tag is visible
     */
    public boolean isTagVisible(int tagId) {
        // OPTIMIZATION: Use simple loop instead of stream
        List<PhotonTrackedTarget> targets = getVisibleTargets();
        for (int i = 0; i < targets.size(); i++) {
            if (targets.get(i).getFiducialId() == tagId) {
                return true;
            }
        }
        return false;
    }
    
    /** Cached best target to avoid recomputing every call */
    private Optional<PhotonTrackedTarget> cachedBestTarget = Optional.empty();
    private double cachedBestTargetTimestamp = 0.0;
    private static final double BEST_TARGET_CACHE_DURATION = 0.05; // Cache for 50ms
    
    /**
     * Gets the best (lowest ambiguity) currently visible target.
     * OPTIMIZED: Caches result to avoid expensive stream operations.
     * 
     * @return Optional containing the best target if any are visible
     */
    public Optional<PhotonTrackedTarget> getBestTarget() {
        double currentTime = Timer.getFPGATimestamp();
        
        // OPTIMIZATION: Return cached result if still valid
        if (cachedBestTarget.isPresent() && 
            (currentTime - cachedBestTargetTimestamp) < BEST_TARGET_CACHE_DURATION) {
            return cachedBestTarget;
        }
        
        // OPTIMIZATION: Only recompute if we have targets
        if (!hasTargets()) {
            cachedBestTarget = Optional.empty();
            return cachedBestTarget;
        }
        
        // OPTIMIZATION: Use simple loop instead of stream for better performance
        List<PhotonTrackedTarget> targets = latestResult.getTargets();
        if (targets.isEmpty()) {
            cachedBestTarget = Optional.empty();
            return cachedBestTarget;
        }
        
        PhotonTrackedTarget best = targets.get(0);
        double bestAmbiguity = best.getPoseAmbiguity();
        
        for (int i = 1; i < targets.size(); i++) {
            PhotonTrackedTarget target = targets.get(i);
            if (target.getPoseAmbiguity() < bestAmbiguity) {
                best = target;
                bestAmbiguity = target.getPoseAmbiguity();
            }
        }
        
        cachedBestTarget = Optional.of(best);
        cachedBestTargetTimestamp = currentTime;
        return cachedBestTarget;
    }
    
    /**
     * Checks if the camera is currently connected and receiving data.
     * 
     * @return true if camera is healthy
     */
    public boolean isCameraConnected() {
        return cameraConnected && (Timer.getFPGATimestamp() - lastResultTimestamp < 1.0);
    }
    
    // ==========================================================================
    // VALIDATION AND HEALTH MONITORING
    // ==========================================================================
    
    /**
     * Validates the quality of a pose estimate.
     * 
     * @param estimate The pose estimate to validate
     * @return true if the estimate meets quality standards
     */
    private boolean isPoseEstimateValid(EstimatedRobotPose estimate) {
        if (estimate.targetsUsed.isEmpty()) {
            return false;
        }
        
        // Check ambiguity for single-tag estimates
        if (estimate.targetsUsed.size() == 1) {
            PhotonTrackedTarget target = estimate.targetsUsed.get(0);
            if (target.getPoseAmbiguity() > Constants.Vision.MAX_POSE_AMBIGUITY) {
                return false;
            }
        }
        
        // Check if pose is within reasonable field boundaries
        Pose3d pose = estimate.estimatedPose;
        if (pose.getX() < -1.0 || pose.getX() > Constants.Field.LENGTH_METERS + 1.0 ||
            pose.getY() < -1.0 || pose.getY() > Constants.Field.WIDTH_METERS + 1.0) {
            return false;
        }
        
        // Check for reasonable height (should be near ground level)
        if (Math.abs(pose.getZ()) > 0.5) {
            return false;
        }
        
        return true;
    }
    
    /**
     * Monitors camera health and connection status.
     */
    private void monitorHealth() {
        // Update connection status based on recent data
        if (Timer.getFPGATimestamp() - lastResultTimestamp > 2.0) {
            if (cameraConnected) {
                logVisionEvents.append("Camera Disconnected (Timeout)");
            }
            cameraConnected = false;
        }
        
        // Log warnings for poor connection
        if (consecutiveFailures > 10 && consecutiveFailures % 50 == 0) {
            logVisionEvents.append("Warning: " + consecutiveFailures + " consecutive camera failures");
        }
    }
    
    // ==========================================================================
    // TELEMETRY AND LOGGING
    // ==========================================================================
    
    /** Cached telemetry strings to avoid repeated formatting */
    private String cachedPoseString = "None";
    private int cachedTagsUsed = 0;
    private int cachedBestTargetId = -1;
    
    /**
     * Updates telemetry data to the dashboard.
     * OPTIMIZED: Throttled updates, cached strings, minimal SmartDashboard calls.
     */
    private void updateTelemetry() {
        // OPTIMIZATION: Batch updates and cache expensive operations
        // Connection status (fast)
        boolean connected = isCameraConnected();
        connectedEntry.setBoolean(connected);
        failuresEntry.setInteger(consecutiveFailures);
        
        // Target information (fast)
        boolean hasTargets = hasTargets();
        int targetCount = hasTargets ? latestResult.getTargets().size() : 0;
        hasTargetsEntry.setBoolean(hasTargets);
        targetCountEntry.setInteger(targetCount);
        
        // Performance metrics (fast)
        frameCountEntry.setInteger(frameCount);
        
        // OPTIMIZATION: Only update pose string if it changed
        if (latestEstimatedPose.isPresent()) {
            Pose2d pose = latestEstimatedPose.get().estimatedPose.toPose2d();
            int tagsUsed = latestEstimatedPose.get().targetsUsed.size();
            
            // Only format string if values changed (avoid expensive String.format)
            if (tagsUsed != cachedTagsUsed || 
                Math.abs(pose.getX()) > 0.01 || Math.abs(pose.getY()) > 0.01) {
                cachedPoseString = String.format("(%.2f, %.2f, %.1f°)", 
                    pose.getX(), pose.getY(), pose.getRotation().getDegrees());
                cachedTagsUsed = tagsUsed;
            }
            estimatedPoseEntry.setString(cachedPoseString);
            tagsUsedEntry.setInteger(cachedTagsUsed);
        } else {
            if (!cachedPoseString.equals("None")) {
                cachedPoseString = "None";
                cachedTagsUsed = 0;
                estimatedPoseEntry.setString(cachedPoseString);
                tagsUsedEntry.setInteger(0);
            }
        }
        
        // OPTIMIZATION: Only update best target if it changed
        Optional<PhotonTrackedTarget> bestTarget = getBestTarget();
        if (bestTarget.isPresent()) {
            PhotonTrackedTarget target = bestTarget.get();
            int targetId = target.getFiducialId();
            if (targetId != cachedBestTargetId) {
                cachedBestTargetId = targetId;
                bestTargetIdEntry.setInteger(targetId);
                bestTargetAmbiguityEntry.setDouble(target.getPoseAmbiguity());
                bestTargetAreaEntry.setDouble(target.getArea());
            }
        } else {
            if (cachedBestTargetId != -1) {
                cachedBestTargetId = -1;
                bestTargetIdEntry.setInteger(-1);
            }
        }
    }
    
    /**
     * Logs detailed pose estimate information.
     * 
     * @param estimate The pose estimate to log
     */
    private void logPoseEstimate(EstimatedRobotPose estimate) {
        // Always log to DataLog for analysis, even if console debug is off
        logPoseX.append(estimate.estimatedPose.getX());
        logPoseY.append(estimate.estimatedPose.getY());
        logPoseTheta.append(estimate.estimatedPose.getRotation().getZ());

        if (Constants.Logging.DEBUG_ENABLED) {
            System.out.println(String.format(
                "[VisionSubsystem] Pose estimate: (%.3f, %.3f, %.1f°) using %d tags",
                estimate.estimatedPose.getX(),
                estimate.estimatedPose.getY(),
                estimate.estimatedPose.getRotation().getZ() * 180.0 / Math.PI,
                estimate.targetsUsed.size()
            ));
        }
    }
    
    /**
     * Gets diagnostic information about the vision subsystem.
     * 
     * @return formatted diagnostic string
     */
    public String getDiagnostics() {
        return String.format(
            "Vision: Connected=%s, Targets=%d, Latency=%.1fms, Failures=%d",
            isCameraConnected(),
            getTargetCount(),
            averageLatencyMs,
            consecutiveFailures
        );
    }
}
