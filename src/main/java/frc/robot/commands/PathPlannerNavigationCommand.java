package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Optimized PathPlanner navigation command following FRC best practices.
 * 
 * Key optimizations:
 * - Pre-computes path in constructor to avoid blocking initialize()
 * - Uses cached command to prevent expensive pathfinding during execution
 * - Adds timing diagnostics for performance monitoring
 */
public class PathPlannerNavigationCommand extends Command {
    private Command inner;
    private final String navigationName;
    private final Pose2d targetPose;
    private final PathConstraints constraints;
    private double initStartTime;
    private boolean pathComputed = false;

    public PathPlannerNavigationCommand(
            CommandSwerveDrivetrain drivetrain,
            Pose2d targetPose,
            PathConstraints constraints,
            String name) {

        this.navigationName = name;
        this.targetPose = targetPose;
        this.constraints = constraints;

        addRequirements(drivetrain);
        
        // OPTIMIZATION: Pre-compute path in constructor (non-blocking for robot loop)
        // This moves the expensive pathfinding out of initialize() which runs in robot loop
        // Note: PathPlanner pathfinding is still fast enough that this won't block startup
        try {
            this.inner = AutoBuilder.pathfindToPose(targetPose, constraints);
            this.pathComputed = true;
        } catch (Exception e) {
            // If pathfinding fails in constructor, defer to initialize
            System.err.println("[PathPlannerNavigation] Warning: Path computation failed in constructor, deferring to initialize: " + e.getMessage());
            this.inner = Commands.defer(
                () -> AutoBuilder.pathfindToPose(targetPose, constraints),
                Set.of(drivetrain)
            );
            this.pathComputed = false;
        }
    }

    @Override 
    public void initialize() { 
        initStartTime = Timer.getFPGATimestamp();
        
        // If path wasn't pre-computed, compute it now (fallback)
        if (!pathComputed && inner == null) {
            double pathStartTime = Timer.getFPGATimestamp();
            try {
                inner = AutoBuilder.pathfindToPose(targetPose, constraints);
                double pathTime = (Timer.getFPGATimestamp() - pathStartTime) * 1000.0;
                System.out.println(String.format(
                    "[PathPlannerNavigation] Path computed in %.2f ms to %s -> %s", 
                    pathTime, navigationName, targetPose));
            } catch (Exception e) {
                System.err.println("[PathPlannerNavigation] Failed to compute path: " + e.getMessage());
                e.printStackTrace();
            }
        }
        
        if (inner != null) {
            inner.initialize();
            double initTime = (Timer.getFPGATimestamp() - initStartTime) * 1000.0;
            if (initTime > 50.0) { // Warn if initialization takes >50ms
                System.out.println(String.format(
                    "[PathPlannerNavigation] WARNING: Initialize took %.2f ms (target: <20ms)", initTime));
            }
            System.out.println(String.format(
                "[PathPlannerNavigation] Starting to %s -> %s (init: %.2f ms)", 
                navigationName, targetPose, initTime));
        }
    }
    
    @Override 
    public void execute() { 
        if (inner != null) {
            inner.execute(); 
        }
    }
    
    @Override 
    public boolean isFinished() { 
        return inner == null || inner.isFinished(); 
    }
    
    @Override 
    public void end(boolean interrupted) {
        if (inner != null) {
            inner.end(interrupted);
        }
        System.out.println(String.format(
            "[PathPlannerNavigation] %s %s", 
            interrupted ? "Interrupted en route to" : "Arrived at", 
            navigationName));
    }
}
