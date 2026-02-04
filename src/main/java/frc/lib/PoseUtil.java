package frc.lib;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class PoseUtil {
    public static enum FieldZone {
        FIELD(Inches.of(651.22), Inches.of(317.69)),
        ALLIANCE(Inches.of(156.61), Inches.of(317.69));

        public final Distance width, height;

        private FieldZone(Distance width, Distance height) {
            this.width = width;
            this.height = height;
        }
    }

    public static final Pose2d flipPoseToOtherAlliance(Pose2d pose) {
        return new Pose2d(
                FieldZone.FIELD.width.baseUnitMagnitude() - pose.getX(),
                FieldZone.FIELD.height.baseUnitMagnitude() - pose.getY(),
                pose.getRotation().plus(new Rotation2d(Math.PI))
        );
    }

    public static final boolean isPoseInAllianceZone(Alliance alliance, Pose2d pose) {
        if (alliance == Alliance.Red) {
            pose = flipPoseToOtherAlliance(pose);
        }
        return pose.getX() <= FieldZone.ALLIANCE.width.baseUnitMagnitude();
    }
}
