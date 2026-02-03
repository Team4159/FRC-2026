package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class PoseUtil {
    private static enum FieldZones {
        FIELD(Units.inchesToMeters(651.22), Units.inchesToMeters(317.69)),
        ALLIANCE(Units.inchesToMeters(156.61), Units.inchesToMeters(317.69));

        public final double width, height;

        private FieldZones(double width, double height) {
            this.width = width;
            this.height = height;
        }
    }

    public static final Pose2d flipPoseToOtherAlliance(Pose2d pose) {
        return new Pose2d(
                FieldZones.FIELD.width - pose.getX(),
                FieldZones.FIELD.height - pose.getY(),
                pose.getRotation().plus(new Rotation2d(Math.PI))
        );
    }

    public static final boolean isPoseInAllianceZone(Alliance alliance, Pose2d pose) {
        if (alliance == Alliance.Red) {
            pose = flipPoseToOtherAlliance(pose);
        }
        return pose.getX() <= FieldZones.ALLIANCE.width;
    }
}
