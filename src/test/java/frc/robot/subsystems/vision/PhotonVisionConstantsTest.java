package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class PhotonVisionConstantsTest {

    @Test
    void cameraTransformsAreMirroredAcrossRobotCenterline() {
        var left = PhotonVisionConstants.LEFT_SHOOTER_CAMERA_TRANSFORM;
        var right = PhotonVisionConstants.RIGHT_SHOOTER_CAMERA_TRANSFORM;

        assertEquals(left.getX(), right.getX(), 1e-9);
        assertEquals(left.getZ(), right.getZ(), 1e-9);
        assertEquals(-left.getY(), right.getY(), 1e-9);
        assertEquals(-left.getRotation().getZ(), right.getRotation().getZ(), 1e-9);
    }

    @Test
    void multiTagMeasurementsHaveLowerPositionUncertainty() {
        assertTrue(PhotonVisionConstants.POSE_AMBIGUITY_THRESHOLD > 0.0);
        assertTrue(PhotonVisionConstants.MULTI_TAG_STANDARD_DEVIATION.get(0, 0) <
            PhotonVisionConstants.SINGLE_TAG_STANDARD_DEVIATION.get(0, 0));
        assertTrue(PhotonVisionConstants.MULTI_TAG_STANDARD_DEVIATION.get(1, 0) <
            PhotonVisionConstants.SINGLE_TAG_STANDARD_DEVIATION.get(1, 0));
    }
}
