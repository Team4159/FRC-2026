package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.FieldZone;
import java.util.Optional;

public final class PoseUtil {

    public static Pose2d flipPoseAlongMiddleXY(Pose2d pose) {
        return new Pose2d(
            FieldZone.FIELD.SIZE.getX() - pose.getX(),
            FieldZone.FIELD.SIZE.getY() - pose.getY(),
            pose.getRotation().plus(Rotation2d.k180deg)
        );
    }

    public static Pose2d flipPoseAlongMiddleY(Pose2d pose) {
        return new Pose2d(pose.getX(), FieldZone.FIELD.SIZE.getY() - pose.getY(), pose.getRotation().times(-1));
    }

    public static Pose2d flipPoseAlongMiddleX(Pose2d pose) {
        return new Pose2d(
            FieldZone.FIELD.SIZE.getX() - pose.getX(),
            pose.getY(),
            Rotation2d.k180deg.minus(pose.getRotation())
        );
    }

    public static boolean isPoseInAllianceZone(Alliance alliance, Pose2d pose) {
        if (alliance == Alliance.Red) {
            pose = flipPoseAlongMiddleXY(pose);
        }
        return pose.getX() <= FieldConstants.ALLIANCE_WIDTH.baseUnitMagnitude();
    }

    public static boolean isPoseBehindAllianceTrenches(Alliance alliance, Pose2d pose) {
        if (alliance == Alliance.Red) {
            pose = flipPoseAlongMiddleXY(pose);
        }
        return pose.getX() <= FieldConstants.TRENCH_X.baseUnitMagnitude();
    }

    public static boolean isPoseOnLeft(Pose2d pose) {
        return pose.getY() >= FieldConstants.ALLIANCE_HEIGHT.baseUnitMagnitude() / 2;
    }

    public static boolean isPoseOnRight(Pose2d pose) {
        return !isPoseOnLeft(pose);
    }

    public static Optional<FieldZone> getPoseTrenchZone(Pose2d pose) {
        for (FieldZone trenchZone : FieldZone.TRENCH_ZONES) {
            boolean nearX = pose
                .getMeasureX()
                .isNear(trenchZone.CENTER.getMeasureX(), trenchZone.SIZE.getMeasureX().div(2));
            boolean nearY = pose
                .getMeasureY()
                .isNear(trenchZone.CENTER.getMeasureY(), trenchZone.SIZE.getMeasureY().div(2));
            if (nearX && nearY) {
                return Optional.of(trenchZone);
            }
        }
        return Optional.empty();
    }
}
