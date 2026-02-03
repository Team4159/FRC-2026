package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class PoseUtil {
    public static final Pose2d fieldFieldRelativePose(Pose2d pose) {
        return new Pose2d(
                Units.inchesToMeters(651.22) - pose.getX(),
                Units.inchesToMeters(317.69) - pose.getY(),
                pose.getRotation().plus(new Rotation2d(Math.PI))
        );
    }
}
