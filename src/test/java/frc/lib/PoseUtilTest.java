package frc.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.FieldZone;
import org.junit.jupiter.api.Test;

class PoseUtilTest {

    @Test
    void flippingAcrossFieldMiddleIsAnInvolution() {
        Pose2d original = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(25));
        Pose2d flippedTwice = PoseUtil.flipPoseAlongMiddleXY(PoseUtil.flipPoseAlongMiddleXY(original));

        assertEquals(original.getX(), flippedTwice.getX(), 1e-9);
        assertEquals(original.getY(), flippedTwice.getY(), 1e-9);
        assertEquals(original.getRotation().getRadians(), flippedTwice.getRotation().getRadians(), 1e-9);
    }

    @Test
    void allianceZoneAndTrenchChecksMirrorForRedAlliance() {
        Pose2d blueZonePose = new Pose2d(
            FieldConstants.ALLIANCE_WIDTH.baseUnitMagnitude() - 0.1,
            2.0,
            new Rotation2d()
        );
        Pose2d redEquivalent = PoseUtil.flipPoseAlongMiddleXY(blueZonePose);

        assertTrue(PoseUtil.isPoseInAllianceZone(Alliance.Blue, blueZonePose));
        assertTrue(PoseUtil.isPoseBehindAllianceTrenches(Alliance.Blue, blueZonePose));
        assertTrue(PoseUtil.isPoseInAllianceZone(Alliance.Red, redEquivalent));
        assertTrue(PoseUtil.isPoseBehindAllianceTrenches(Alliance.Red, redEquivalent));
        assertFalse(PoseUtil.isPoseOnRight(new Pose2d(0, FieldConstants.ALLIANCE_HEIGHT.baseUnitMagnitude() / 2, new Rotation2d())));
    }

    @Test
    void findsAConfiguredTrenchZoneAtItsCenter() {
        FieldZone zone = FieldConstants.TRENCH_ZONES[0];
        assertEquals(zone, PoseUtil.getPoseTrenchZone(new Pose2d(zone.CENTER, new Rotation2d())).orElseThrow());
    }
}
