package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimpleAutonomousCommand {
    // Use 40% of max speed for safe testing (matches test commands in RobotContainer)
    private static final double TEST_SPEED_SCALE = 0.4;
    private static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double TEST_VELOCITY = MAX_SPEED * TEST_SPEED_SCALE;
    private static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private static final double TEST_ANGULAR_RATE = MAX_ANGULAR_RATE * TEST_SPEED_SCALE;
    
    public static Command driveForward(CommandSwerveDrivetrain drivetrain, double seconds) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] driveForward: Starting - Target velocity: " + TEST_VELOCITY + " m/s (" + (TEST_SPEED_SCALE * 100) + "% of max " + MAX_SPEED + " m/s)");
                SmartDashboard.putString("SimpleAuto/Status", "Driving Forward");
                SmartDashboard.putNumber("SimpleAuto/TargetVelocity", TEST_VELOCITY);
            }),
            drivetrain.applyRequest(() -> {
                var request = new SwerveRequest.RobotCentric()
                    .withVelocityX(TEST_VELOCITY)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0);
                
                // Log actual speeds periodically
                var state = drivetrain.getState();
                SmartDashboard.putNumber("SimpleAuto/ActualVelocityX", state.Speeds.vxMetersPerSecond);
                SmartDashboard.putNumber("SimpleAuto/ActualVelocityY", state.Speeds.vyMetersPerSecond);
                
                return request;
            }).withTimeout(seconds),
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] driveForward: Complete");
                SmartDashboard.putString("SimpleAuto/Status", "Complete");
            })
        );
    }

    public static Command spinInPlace(CommandSwerveDrivetrain drivetrain, double seconds) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] spinInPlace: Starting - Target angular rate: " + TEST_ANGULAR_RATE + " rad/s (" + (TEST_SPEED_SCALE * 100) + "% of max " + MAX_ANGULAR_RATE + " rad/s)");
                SmartDashboard.putString("SimpleAuto/Status", "Spinning In Place");
                SmartDashboard.putNumber("SimpleAuto/TargetAngularRate", TEST_ANGULAR_RATE);
            }),
            drivetrain.applyRequest(() -> {
                var request = new SwerveRequest.RobotCentric()
                    .withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(TEST_ANGULAR_RATE);
                
                // Log actual speeds periodically
                var state = drivetrain.getState();
                SmartDashboard.putNumber("SimpleAuto/ActualAngularRate", state.Speeds.omegaRadiansPerSecond);
                
                return request;
            }).withTimeout(seconds),
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] spinInPlace: Complete");
                SmartDashboard.putString("SimpleAuto/Status", "Complete");
            })
        );
    }

    public static Command squarePattern(CommandSwerveDrivetrain drivetrain) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] squarePattern: Starting - Using velocity: " + TEST_VELOCITY + " m/s");
                SmartDashboard.putString("SimpleAuto/Status", "Square Pattern");
            }),
            // Forward
            Commands.runOnce(() -> System.out.println("[SimpleAuto] squarePattern: Moving Forward")),
            drivetrain.applyRequest(() -> 
                new SwerveRequest.RobotCentric()
                    .withVelocityX(TEST_VELOCITY)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0)
            ).withTimeout(1.0),
            // Right
            Commands.runOnce(() -> System.out.println("[SimpleAuto] squarePattern: Moving Right")),
            drivetrain.applyRequest(() -> 
                new SwerveRequest.RobotCentric()
                    .withVelocityX(0.0)
                    .withVelocityY(-TEST_VELOCITY)
                    .withRotationalRate(0.0)
            ).withTimeout(1.0),
            // Back
            Commands.runOnce(() -> System.out.println("[SimpleAuto] squarePattern: Moving Backward")),
            drivetrain.applyRequest(() -> 
                new SwerveRequest.RobotCentric()
                    .withVelocityX(-TEST_VELOCITY)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0)
            ).withTimeout(1.0),
            // Left
            Commands.runOnce(() -> System.out.println("[SimpleAuto] squarePattern: Moving Left")),
            drivetrain.applyRequest(() -> 
                new SwerveRequest.RobotCentric()
                    .withVelocityX(0.0)
                    .withVelocityY(TEST_VELOCITY)
                    .withRotationalRate(0.0)
            ).withTimeout(1.0),
            Commands.runOnce(() -> {
                System.out.println("[SimpleAuto] squarePattern: Complete");
                SmartDashboard.putString("SimpleAuto/Status", "Complete");
            })
        );
    }
}
