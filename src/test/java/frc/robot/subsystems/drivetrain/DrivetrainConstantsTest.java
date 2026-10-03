package frc.robot.subsystems.drivetrain;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class DrivetrainConstantsTest {

    @Test
    void motionLimitsAreValid() {
        assertTrue(DrivetrainConstants.MAX_TRANSLATION_SPEED > 0.0);
        assertTrue(DrivetrainConstants.MAX_ROTATION_SPEED > 0.0);
    }
}
