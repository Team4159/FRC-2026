package frc.lib;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.FieldZone;

public final class PoseUtil {

    public static final Pose2d flipPoseAlongMiddleXY(Pose2d pose) {
        return new Pose2d(
                FieldZone.FIELD.size.getX() - pose.getX(),
                FieldZone.FIELD.size.getY() - pose.getY(),
                pose.getRotation().plus(Rotation2d.k180deg)
        );
    }

    public static final Pose2d flipPoseAlongMiddleY(Pose2d pose) {
        return new Pose2d(
                pose.getX(),
                FieldZone.FIELD.size.getY() - pose.getY(),
                pose.getRotation().times(-1)
        );
    }

    public static final Pose2d flipPoseAlongMiddleX(Pose2d pose) {
        return new Pose2d(
                FieldZone.FIELD.size.getX() - pose.getX(),
                pose.getY(),
                Rotation2d.k180deg.minus(pose.getRotation())
        );
    }

    public static final boolean isPoseInAllianceZone(Alliance alliance, Pose2d pose) {
        if (alliance == Alliance.Red) {
            pose = flipPoseAlongMiddleXY(pose);
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
