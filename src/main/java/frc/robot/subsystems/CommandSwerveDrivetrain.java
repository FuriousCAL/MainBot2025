package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.constants.Constants;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final GenericEntry reqVelXEntry;
    private final GenericEntry reqVelYEntry;
    private final GenericEntry reqAngRateEntry;
    private final GenericEntry actVelXEntry;
    private final GenericEntry actVelYEntry;
    private final GenericEntry actAngRateEntry;
    private final GenericEntry poseXEntry;
    private final GenericEntry poseYEntry;
    private final GenericEntry poseRotEntry;
    private final GenericEntry poseStringEntry; // New text box for easy reading

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        
        ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
        reqVelXEntry = driveTab.add("Req Vel X", 0.0).getEntry();
        reqVelYEntry = driveTab.add("Req Vel Y", 0.0).getEntry();
        reqAngRateEntry = driveTab.add("Req Ang Rate", 0.0).getEntry();
        actVelXEntry = driveTab.add("Act Vel X", 0.0).getEntry();
        actVelYEntry = driveTab.add("Act Vel Y", 0.0).getEntry();
        actAngRateEntry = driveTab.add("Act Ang Rate", 0.0).getEntry();
        poseXEntry = driveTab.add("Pose X", 0.0).getEntry();
        poseYEntry = driveTab.add("Pose Y", 0.0).getEntry();
        poseRotEntry = driveTab.add("Pose Rot", 0.0).getEntry();
        
        // Add formatted text box
        poseStringEntry = driveTab.add("Current Pose", "Initializing...")
            .withPosition(3, 0)
            .withSize(2, 1) // Make it big enough to read
            .getEntry();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        
        ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
        reqVelXEntry = driveTab.add("Req Vel X", 0.0).getEntry();
        reqVelYEntry = driveTab.add("Req Vel Y", 0.0).getEntry();
        reqAngRateEntry = driveTab.add("Req Ang Rate", 0.0).getEntry();
        actVelXEntry = driveTab.add("Act Vel X", 0.0).getEntry();
        actVelYEntry = driveTab.add("Act Vel Y", 0.0).getEntry();
        actAngRateEntry = driveTab.add("Act Ang Rate", 0.0).getEntry();
        poseXEntry = driveTab.add("Pose X", 0.0).getEntry();
        poseYEntry = driveTab.add("Pose Y", 0.0).getEntry();
        poseRotEntry = driveTab.add("Pose Rot", 0.0).getEntry();
        
        // Add formatted text box
        poseStringEntry = driveTab.add("Current Pose", "Initializing...")
            .withPosition(3, 0)
            .withSize(2, 1)
            .getEntry();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public void resetPose(Pose2d pose) {
        // Using "vision measurement" hack to reset pose since Phoenix 6 SwerveDrivetrain 
        // handles odometry internally. This forces the estimator to the desired pose.
        try { 
            this.setOperatorPerspectiveForward(pose.getRotation());
            
            // Apply high-confidence vision measurement to reset pose
            // standard deviation 0.1 is very low variance = high trust
            this.addVisionMeasurement(pose, Timer.getFPGATimestamp(), VecBuilder.fill(0.1, 0.1, 0.1));
            
        } catch (Throwable t) {
            System.err.println("Failed to reset pose: " + t.getMessage());
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getState().Speeds;
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        // Apply clamps based on SAFETY_TESTING_MODE flag
        double maxSpeed = Constants.Safety.SAFETY_TESTING_MODE ? 
            Constants.Safety.TEST_MAX_SPEED : Constants.Drivetrain.MAX_VELOCITY_MPS;
        double maxRot = Constants.Safety.SAFETY_TESTING_MODE ? 
            Constants.Safety.TEST_MAX_ROTATION : Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADPS;
            
        double clampedVx = MathUtil.clamp(speeds.vxMetersPerSecond, -maxSpeed, maxSpeed);
        double clampedVy = MathUtil.clamp(speeds.vyMetersPerSecond, -maxSpeed, maxSpeed);
        double clampedOmega = MathUtil.clamp(speeds.omegaRadiansPerSecond, -maxRot, maxRot);
        
        // Log velocity requests for debugging
        if (reqVelXEntry != null) { // Check null in case initialized differently (redundant safety)
            reqVelXEntry.setDouble(clampedVx);
            reqVelYEntry.setDouble(clampedVy);
            reqAngRateEntry.setDouble(clampedOmega);
        }
        
        // Select DriveRequestType: Velocity for testing (safer), OpenLoop for match (responsive)
        // Or keep Velocity if tuned well. Using switch based on flag for now.
        var requestType = Constants.Safety.SAFETY_TESTING_MODE ? 
            com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity : 
            com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage;
        
        var request = new SwerveRequest.RobotCentric()
            .withVelocityX(clampedVx)
            .withVelocityY(clampedVy)
            .withRotationalRate(clampedOmega)
            .withDriveRequestType(requestType);
        this.setControl(request);
        
        // Log actual speeds after applying
        var state = this.getState();
        if (actVelXEntry != null) {
            actVelXEntry.setDouble(state.Speeds.vxMetersPerSecond);
            actVelYEntry.setDouble(state.Speeds.vyMetersPerSecond);
            actAngRateEntry.setDouble(state.Speeds.omegaRadiansPerSecond);
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /**
     * Resets the drivetrain's odometry to the specified field pose.
     * This method can be called by PathPlanner before starting a new path.
     *
    @param newPose The desired field pose.
     */
    public void resetOdometry(Pose2d newPose) {
        // Reset internal components to the new pose.
        resetPose(newPose);
        // If additional odometry methods exist (e.g., in the base class), call them here.
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        
        // Log drivetrain state for debugging
        var state = this.getState();
        if (poseXEntry != null) {
            double x = state.Pose.getX();
            double y = state.Pose.getY();
            double rot = state.Pose.getRotation().getDegrees();
            
            poseXEntry.setDouble(x);
            poseYEntry.setDouble(y);
            poseRotEntry.setDouble(rot);
            actVelXEntry.setDouble(state.Speeds.vxMetersPerSecond);
            actVelYEntry.setDouble(state.Speeds.vyMetersPerSecond);
            actAngRateEntry.setDouble(state.Speeds.omegaRadiansPerSecond);
            
            // Update formatted string (less frequent updates? No, string fmt is fast enough for <10 calls)
            poseStringEntry.setString(String.format("(%.2f, %.2f) @ %.1f°", x, y, rot));
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}
