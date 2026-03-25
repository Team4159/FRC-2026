package frc.lib;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.FieldZone;

public final class FieldUtil {

    public static final Pose2d flipPoseToOtherAlliance(Pose2d pose) {
        return new Pose2d(
                FieldZone.FIELD.size.getMeasureX().minus(pose.getMeasureX()),
                FieldZone.FIELD.size.getMeasureY().minus(pose.getMeasureY()),
                pose.getRotation().plus(Rotation2d.k180deg)
        );
    }

    public static final boolean isPoseInAllianceZone(Alliance alliance, Pose2d pose) {
        if (alliance == Alliance.Red) {
            pose = flipPoseToOtherAlliance(pose);
        }
        return pose.getX() <= FieldConstants.kAllianceWidth.baseUnitMagnitude();
    }

    public static final boolean isPoseOnLeft(Pose2d pose) {
        return pose.getY() >= FieldConstants.kAllianceHeight.baseUnitMagnitude() / 2;
    }

    public static final boolean isPoseOnRight(Pose2d pose) {
        return !isPoseOnLeft(pose);
    }

    public static final Optional<FieldZone> getPoseTrenchZone(Pose2d pose) {
        for (FieldZone fieldZone : FieldConstants.kTrenchZones) {
            boolean nearX = pose.getMeasureX().isNear(fieldZone.center.getMeasureX(), fieldZone.size.getMeasureX().div(2));
            boolean nearY = pose.getMeasureY().isNear(fieldZone.center.getMeasureY(), fieldZone.size.getMeasureY().div(2));
            if (nearX && nearY) {
                return Optional.of(fieldZone);
            }
        }
        return Optional.empty();
    }
}
