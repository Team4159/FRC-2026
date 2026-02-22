package frc.lib;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants.FieldZone;
import frc.robot.Constants.FieldConstants.TrenchZone;

public final class FieldUtil {

    public static final Pose2d flipPoseToOtherAlliance(Pose2d pose) {
        return new Pose2d(
                FieldZone.FIELD.width.in(Meters) - pose.getX(),
                FieldZone.FIELD.height.in(Meters) - pose.getY(),
                pose.getRotation().plus(Rotation2d.k180deg)
        );
    }

    public static final boolean isPoseInAllianceZone(Alliance alliance, Pose2d pose) {
        if (alliance == Alliance.Red) {
            pose = flipPoseToOtherAlliance(pose);
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
