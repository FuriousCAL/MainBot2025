package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import frc.robot.commands.DriveToAprilTag2Command;
import frc.robot.commands.DriveToAprilTagOffsetCommand;
import frc.robot.commands.DriveToHomeCommand;
import frc.robot.commands.SimpleAutonomousCommand;
import frc.robot.commands.VisionAssistedAprilTagCommand;
import frc.robot.constants.AprilTagConstants;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Set;
import java.util.Optional;

// Import StartPosition from the new file
import frc.robot.StartPosition;


public class RobotContainer {
  // Speed configuration based on SAFETY_TESTING_MODE flag
  private final double MaxSpeed = Constants.Safety.SAFETY_TESTING_MODE ? 
      Constants.Safety.TEST_MAX_SPEED : TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
      
  private final double MaxAngularRate = Constants.Safety.SAFETY_TESTING_MODE ? 
      RotationsPerSecond.of(0.1).in(RadiansPerSecond) : RotationsPerSecond.of(0.40).in(RadiansPerSecond);

  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(Constants.Safety.SAFETY_TESTING_MODE ? 
          DriveRequestType.Velocity : DriveRequestType.OpenLoopVoltage);
          
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  
  // Field-Centric request
  private final SwerveRequest.FieldCentric fieldDrive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1)
    .withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(Constants.Safety.SAFETY_TESTING_MODE ? 
          DriveRequestType.Velocity : DriveRequestType.OpenLoopVoltage);

  // Start in field-centric; LB will toggle this
  private boolean isFieldCentric = true;

  private final Field2d field = new Field2d();           // Field widget
  private final Telemetry logger;                        // <-- declare only

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final CommandXboxController joystick = new CommandXboxController(0);

  private SendableChooser<Command> autoChooser;
  
  // Match Setup Choosers
  private final SendableChooser<Alliance> teamChooser = new SendableChooser<>();
  private final SendableChooser<StartPosition> startPositionChooser = new SendableChooser<>();
  
  // Track last selected values to avoid repetitive updates
  private Alliance lastAlliance = null;
  private StartPosition lastStartPosition = null;
  
  public RobotContainer() {
    // 1) PathPlanner hooks
    configurePathPlanner();

    // 2) Chooser before adding options
    createAutoChooser();
    
    // 3) Match Setup Configuration
    configureMatchSetup();

    // 4) Expose field widget & create telemetry
    SmartDashboard.putData("Field", field);
    logger = new Telemetry(MaxSpeed, field);            // <-- construct once

    // 5) Setup vision pose fusion
    configureVisionFusion();

    // 6) Default teleop drive
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() -> {
         // Don't run default command in test mode
         if (edu.wpi.first.wpilibj.DriverStation.isTest()) {
           return brake; // Return brake request in test mode to prevent conflicts
         }
         
         // Trigger-based speed scaling
        double precision = 1.0 - 0.7 * joystick.getLeftTriggerAxis();  // 1.0 → 0.3
        double turbo     = 1.0 + 0.5 * joystick.getRightTriggerAxis(); // 1.0 → 1.5
        double scale     = MathUtil.clamp(precision * turbo, 0.2, 1.5);
        
        // If in safe mode, disable turbo/precision scaling to keep things predictable
        if (Constants.Safety.SAFETY_TESTING_MODE) {
            scale = 1.0;
        }

        // Call the concrete request type (avoid the NativeSwerveRequest parent)
        double rightX = joystick.getRightX();
        double rotationRate = -rightX * MaxAngularRate * scale; // Invert for correct rotation direction
        
        // Debug: Print rotation values
        if (Math.abs(rightX) > 0.1) {
          System.out.println("Right stick X: " + rightX + ", Rotation rate: " + rotationRate);
        }
        
        if (isFieldCentric) {
          return fieldDrive
              .withVelocityX(joystick.getLeftY()  * -MaxSpeed         * scale)
              .withVelocityY(joystick.getLeftX()  * -MaxSpeed         * scale)
              .withRotationalRate(rotationRate);
        } else {
          return drive
              .withVelocityX(joystick.getLeftY()  * -MaxSpeed         * scale)
              .withVelocityY(joystick.getLeftX()  * -MaxSpeed         * scale)
              .withRotationalRate(rotationRate);
        }
      })
      .withName("DefaultTeleopDrive")
    );

    // 7) Buttons and SysId
    configureBindings();

    // 8) Add custom auto options
    populateAutoChooser();

    // 9) Feed telemetry (this updates Field2d via Telemetry.telemeterize)
    drivetrain.registerTelemetry(logger::telemeterize);
  }
  
  /**
   * Configure the Match Setup tab on Shuffleboard.
   * This allows the driver to select the alliance team and starting position.
   */
  private void configureMatchSetup() {
      ShuffleboardTab matchTab = Shuffleboard.getTab("Match Setup");
      
      // Team Chooser
      teamChooser.setDefaultOption("Blue Alliance", Alliance.Blue);
      teamChooser.addOption("Red Alliance", Alliance.Red);
      matchTab.add("Team Selection", teamChooser)
              .withPosition(0, 0)
              .withSize(2, 1);
      
      // Start Position Chooser
      startPositionChooser.setDefaultOption(StartPosition.MID.displayName, StartPosition.MID);
      startPositionChooser.addOption(StartPosition.TOP.displayName, StartPosition.TOP);
      startPositionChooser.addOption(StartPosition.LOW.displayName, StartPosition.LOW);
      matchTab.add("Start Position", startPositionChooser)
              .withPosition(2, 0)
              .withSize(2, 1);
              
      // Button to reset pose based on selection
      matchTab.add("SET START POSE", Commands.runOnce(() -> resetPoseToMatchSetup(true)))
              .withPosition(4, 0)
              .withSize(2, 1);
  }

  /**
   * Resets the robot's pose based on the Match Setup selections.
   * This should be called during autonomousInit() or manually via the dashboard button.
   */
  public void resetPoseToMatchSetup() {
      resetPoseToMatchSetup(false);
  }

  /**
   * Resets the robot's pose based on the Match Setup selections.
   * 
   * @param forceReset If true, resets pose even if selection hasn't changed.
   */
  public void resetPoseToMatchSetup(boolean forceReset) {
      Alliance selectedAlliance = teamChooser.getSelected();
      StartPosition selectedPos = startPositionChooser.getSelected();
      
      // Fallback to defaults if dashboard hasn't connected/sent values yet
      if (selectedAlliance == null) {
          selectedAlliance = Alliance.Blue;
      }
      if (selectedPos == null) {
          selectedPos = StartPosition.MID;
      }
      
      // Check if selection changed (unless forcing reset)
      if (!forceReset && selectedAlliance == lastAlliance && selectedPos == lastStartPosition) {
          return; // No change, do nothing
      }
      
      // Update last state
      lastAlliance = selectedAlliance;
      lastStartPosition = selectedPos;
      
      // Get base pose (Blue alliance relative)
      Pose2d startPose = selectedPos.bluePose;
      
      // If Red alliance, flip the pose to the other side of the field
      if (selectedAlliance == Alliance.Red) {
          // Flip X coordinate: FieldLength - X
          // Flip Rotation: 180 - Rotation
          // Y coordinate remains the same (mirrored field)
          double fieldLength = AprilTagConstants.FIELD_LAYOUT.getFieldLength();
          double flippedX = fieldLength - startPose.getX();
          Rotation2d flippedRot = Rotation2d.fromDegrees(180).minus(startPose.getRotation());
          
          startPose = new Pose2d(flippedX, startPose.getY(), flippedRot);
      }
      
      // Apply the pose reset
      drivetrain.resetPose(startPose);
      System.out.println("[MatchSetup] Pose reset to " + selectedAlliance + " " + selectedPos.name() + ": " + startPose);
  }

  /**
   * Configure vision pose fusion to improve PathPlanner accuracy.
   * This is the missing piece that connects VisionSubsystem to CommandSwerveDrivetrain.
   */
  private void configureVisionFusion() {
    // This will be called in periodic() to continuously fuse vision measurements
    System.out.println("[VisionFusion] Vision pose fusion configured");
  }

  /**
   * This should be called periodically from Robot.java to fuse vision measurements.
   */
  public void updateVisionFusion() {
    // Continuously fuse vision poses with drivetrain odometry
    visionSubsystem.getLatestEstimatedPose().ifPresent(estimatedPose -> {
      // Add vision measurement to drivetrain's pose estimator
      drivetrain.addVisionMeasurement(
        estimatedPose.estimatedPose.toPose2d(),
        estimatedPose.timestampSeconds,
        visionSubsystem.getVisionMeasurementStdDevs()
      );
    });
  }

  private void configureBindings() {
    // ============================================================================
    // PROFESSIONAL FRC TEAM CONTROLLER LAYOUT (Following 254/971/1678 patterns)
    // ============================================================================
    
    // === CORE DRIVING CONTROLS ===
    // Left Bumper: Toggle Field/Robot Centric (industry standard)
    joystick.leftBumper().onTrue(Commands.runOnce(() -> {
        isFieldCentric = !isFieldCentric;
        if (isFieldCentric) {
            drivetrain.seedFieldCentric();
        }
    }));

    // Right Bumper: Brake mode (hold to brake, industry standard)
    joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));

    // A Button: Cancel all commands and return to manual control
    joystick.a().onTrue(Commands.sequence(
        drivetrain.runOnce(() -> {}), // Cancel any active commands
        drivetrain.applyRequest(() -> brake).withTimeout(0.25)
    ));

    // B Button: Point wheels toward left stick direction (precision alignment)
    joystick.b().whileTrue(
        drivetrain.applyRequest(() -> {
            double x = -joystick.getLeftX();
            double y = -joystick.getLeftY();
            double mag = Math.hypot(x, y);
            Rotation2d dir = (mag > 0.10)
                ? new Rotation2d(Math.atan2(y, x))
                : drivetrain.getState().Pose.getRotation();
            return point.withModuleDirection(dir);
        })
    );

    // === VISION-ASSISTED NAVIGATION (Button Combinations) ===
    // X + Y: Drive to AprilTag 2 (Speaker, main scoring position)
    joystick.x().and(joystick.y()).onTrue(new DriveToAprilTag2Command(drivetrain, visionSubsystem));
    
    // X + B: Drive to AprilTag 1 (Blue alliance scoring)
    joystick.x().and(joystick.b()).onTrue(new VisionAssistedAprilTagCommand(drivetrain, visionSubsystem, 1));
    
    // Y + B: Drive to AprilTag 3 (Amp side)
    joystick.y().and(joystick.b()).onTrue(new VisionAssistedAprilTagCommand(drivetrain, visionSubsystem, 3));
    
    // X + A: Drive to AprilTag 4 (Source side)
    joystick.x().and(joystick.a()).onTrue(new VisionAssistedAprilTagCommand(drivetrain, visionSubsystem, 4));
    
    // === AUTOMATED FIELD NAVIGATION (Red/Blue Aware) ===
    // Use POV/D-pad to trigger specific tags based on alliance
    
    // D-Pad Up: Speaker Center (Tag 7 for Blue, Tag 4 for Red)
    joystick.povUp().onTrue(Commands.defer(() -> {
        var alliance = DriverStation.getAlliance();
        int tagId = 7; // Default Blue Speaker (7)
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            tagId = 4; // Red Speaker (4)
        }
        return new VisionAssistedAprilTagCommand(drivetrain, visionSubsystem, tagId);
    }, Set.of(drivetrain, visionSubsystem)));
    
    // D-Pad Left: Amp (Tag 6 for Blue, Tag 5 for Red)
    joystick.povLeft().onTrue(Commands.defer(() -> {
        var alliance = DriverStation.getAlliance();
        int tagId = 6; // Default Blue Amp (6)
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            tagId = 5; // Red Amp (5)
        }
        return new VisionAssistedAprilTagCommand(drivetrain, visionSubsystem, tagId);
    }, Set.of(drivetrain, visionSubsystem)));
    
    // D-Pad Right: Source (Tag 1 for Blue, Tag 10 for Red)
    // Note: Choosing the source tag closest to the alliance wall for simplicity
    joystick.povRight().onTrue(Commands.defer(() -> {
        var alliance = DriverStation.getAlliance();
        int tagId = 1; // Default Blue Source (1 or 2, using 1)
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            tagId = 10; // Red Source (9 or 10, using 10)
        }
        return new VisionAssistedAprilTagCommand(drivetrain, visionSubsystem, tagId);
    }, Set.of(drivetrain, visionSubsystem)));

    // === SAFETY AND UTILITY ===
    // Y Button: Return to home position (safe zone)
    // NOTE: This uses the SAFE_POSE constant directly, not the Shuffleboard-selected start pose
    joystick.y().onTrue(new DriveToHomeCommand(drivetrain));
    
    // Start Button: Velocity Test (Fixed 0.5 m/s forward)
    // Uses Closed Loop Velocity Control to ensure safe, consistent speed on blocks
    joystick.start().whileTrue(drivetrain.applyRequest(() -> 
        new SwerveRequest.RobotCentric()
            .withVelocityX(0.5) // 0.5 m/s forward
            .withVelocityY(0.0)
            .withRotationalRate(0.0)
            .withDriveRequestType(DriveRequestType.Velocity)
    ));
    
    // Back Button: Emergency home (backup safety)
    joystick.back().onTrue(new DriveToHomeCommand(drivetrain));

    // === D-PAD: QUICK NAVIGATION (Optional) ===
    // D-Pad Down: Drive to ALL points (Amp -> Speaker -> Source)
    joystick.povDown().onTrue(Commands.defer(() -> {
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        
        // Define tag IDs based on alliance
        int ampId = isRed ? 5 : 6;
        int speakerId = isRed ? 4 : 7;
        int sourceId = isRed ? 10 : 1;
        
        return Commands.sequence(
            // 1. Go to Amp
            new VisionAssistedAprilTagCommand(drivetrain, visionSubsystem, ampId),
            Commands.waitSeconds(0.5),
            
            // 2. Go to Speaker
            new VisionAssistedAprilTagCommand(drivetrain, visionSubsystem, speakerId),
            Commands.waitSeconds(0.5),
            
            // 3. Go to Source
            new VisionAssistedAprilTagCommand(drivetrain, visionSubsystem, sourceId)
        );
    }, Set.of(drivetrain, visionSubsystem)));

    System.out.println("[Controller] Professional FRC team layout loaded");
    System.out.println("  Core: LB=Toggle Mode, RB=Brake, A=Cancel, B=Point, Y=Home");
    System.out.println("  Vision: X+Y=Tag2, X+B=Tag1, Y+B=Tag3, X+A=Tag4");
    System.out.println("  Red/Blue: POV-Up=Speaker, POV-Left=Amp, POV-Right=Source, POV-Down=ALL POINTS");
    System.out.println("  Safety: Start=Vision Test, Back=Emergency Home");
    System.out.println("  Rotation: Left stick = CCW, Right stick = CW");
  }


  private void configurePathPlanner() {
    // Register NamedCommands to prevent runtime errors
    NamedCommands.registerCommand("ElevatorL1", Commands.print("ElevatorL1 command not implemented"));
    NamedCommands.registerCommand("Shoot", Commands.print("Shoot command not implemented"));
    NamedCommands.registerCommand("ElevatorBottom", Commands.print("ElevatorBottom command not implemented"));
    NamedCommands.registerCommand("ElevatorFeed", Commands.print("ElevatorFeed command not implemented"));

    RobotConfig cfg;
    try {
        cfg = RobotConfig.fromGUISettings();
        System.out.println("[PathPlanner] Successfully loaded config: " + cfg.toString());
    } catch (Exception e) {
        System.err.println("[PathPlanner] Failed to load config: " + e.getMessage());
        e.printStackTrace();
        throw new RuntimeException("Failed to load PathPlanner RobotConfig from GUI settings", e);
    }

    AutoBuilder.configure(
        drivetrain::getPose,
        drivetrain::resetPose,
        drivetrain::getRobotRelativeSpeeds,
        (speeds, ffs) -> drivetrain.driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)
        ),
        cfg,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        drivetrain
    );
    
    System.out.println("[PathPlanner] AutoBuilder configured successfully");
}

  private void createAutoChooser() {
    try {
      autoChooser = AutoBuilder.buildAutoChooser();
    } catch (IllegalStateException ex) {
      autoChooser = new SendableChooser<>();
      System.err.println("[Auto] AutoBuilder not configured, using empty chooser: " + ex.getMessage());
    }
    // Add to specific "Auto" tab
    Shuffleboard.getTab("Auto")
        .add("Auto Chooser", autoChooser)
        .withPosition(0, 0)
        .withSize(2, 1);
        
    // Also keep on SmartDashboard for backward compatibility/ease of access
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void populateAutoChooser() {
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    
    // PathPlanner: Standard Test Auto (Fixed Path)
    // NOTE: Speed limited to 0.5 m/s in TestPath1.path
    try {
        autoChooser.addOption("PathPlanner: TestAuto1", AutoBuilder.buildAuto("TestAuto1"));
    } catch (Exception e) {
        System.err.println("Could not load TestAuto1: " + e.getMessage());
    }

    // ============================================================================
    // VISION TEST AUTONOMOUS COMMANDS (Adaptive with Slowing)
    // ============================================================================
    // Key tags for testing: Speaker (2), Source (N/A?), Amp? 
    // We'll expose Tag 2 (Speaker), Tag 3 (Speaker-side), Tag 4 (Source-side) 
    // These use VisionAssistedAprilTagCommand which includes the drastic slowing logic
    
    autoChooser.addOption("Vision Test: Tag 2 (Speaker)", 
        new VisionAssistedAprilTagCommand(drivetrain, visionSubsystem, 2));
        
    autoChooser.addOption("Vision Test: Tag 3 (Speaker Side)", 
        new VisionAssistedAprilTagCommand(drivetrain, visionSubsystem, 3));
        
    autoChooser.addOption("Vision Test: Tag 4 (Source Side)", 
        new VisionAssistedAprilTagCommand(drivetrain, visionSubsystem, 4));

    System.out.println("[AutoChooser] Configured for safe block testing");
  }

  public Command getAutonomousCommand() {
    return autoChooser != null ? autoChooser.getSelected() : Commands.none();
  }
  
}
