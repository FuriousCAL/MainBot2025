package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// Enum for Start Positions
public enum StartPosition {
    TOP("Top (Speaker/Source)", new Pose2d(0.7, 6.7, Rotation2d.fromDegrees(60))),
    MID("Mid (Speaker Center)", new Pose2d(1.3, 5.5, Rotation2d.fromDegrees(0))),
    LOW("Low (Amp Side)", new Pose2d(0.7, 4.4, Rotation2d.fromDegrees(-60)));
    
    public final String displayName;
    public final Pose2d bluePose; // Pose relative to Blue alliance wall
    
    StartPosition(String displayName, Pose2d bluePose) {
        this.displayName = displayName;
        this.bluePose = bluePose;
    }
}
