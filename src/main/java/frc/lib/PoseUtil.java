package frc.lib;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants.FieldZone;
import frc.robot.Constants.FieldConstants.TrenchZone;

public final class PoseUtil {

    public static final Pose2d flipPoseAlongMiddleXY(Pose2d pose) {
        return new Pose2d(
                FieldZone.FIELD.width.baseUnitMagnitude() - pose.getX(),
                FieldZone.FIELD.height.baseUnitMagnitude() - pose.getY(),
                pose.getRotation().plus(Rotation2d.k180deg)
        );
    }

    public static final Pose2d flipPoseAlongMiddleY(Pose2d pose) {
        return new Pose2d(
                FieldZone.FIELD.width.baseUnitMagnitude(),
                FieldZone.FIELD.height.baseUnitMagnitude() - pose.getY(),
                Rotation2d.k180deg.minus(pose.getRotation())
        );
    }

    public static final Pose2d flipPoseAlongMiddleX(Pose2d pose) {
        return new Pose2d(
                FieldZone.FIELD.width.baseUnitMagnitude() - pose.getX(),
                FieldZone.FIELD.height.baseUnitMagnitude(),
                pose.getRotation().times(-1)
        );
    }

    public static final boolean isPoseInAllianceZone(Alliance alliance, Pose2d pose) {
        if (alliance == Alliance.Red) {
            pose = flipPoseAlongMiddleXY(pose);
        }
        return pose.getX() <= FieldZone.ALLIANCE.width.in(Meters);
    }

    public static final boolean isPoseOnRight(Pose2d pose) {
        return pose.getY() <= FieldZone.ALLIANCE.height.in(Meters);
    }

    public static final boolean isPoseOnLeft(Pose2d pose) {
        return !isPoseOnRight(pose);
    }

    public static final Optional<TrenchZone> getPoseTrenchZone(Pose2d pose) {
        for (TrenchZone zone : TrenchZone.values()) {
            if (pose.getMeasureX().isNear(zone.x, FieldZone.TRENCH.width.div(2)) && pose.getMeasureY().isNear(zone.y, FieldZone.TRENCH.height.div(2))) {
                return Optional.of(zone);
            }
        }
        return Optional.empty();
    }
}
