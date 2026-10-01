package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class DrivetrainConstantsTest {

    @Test
    void dimensionsAndMotionLimitsAreValid() {
        assertEquals(27.0, DrivetrainConstants.CHASSIS_SIZE_X.in(Inches));
        assertEquals(27.0, DrivetrainConstants.CHASSIS_SIZE_Y.in(Inches));
        assertEquals(35.0, DrivetrainConstants.BUMPER_SIZE_X.in(Inches));
        assertEquals(35.0, DrivetrainConstants.BUMPER_SIZE_Y.in(Inches));
        assertTrue(DrivetrainConstants.MAX_TRANSLATION_SPEED > 0.0);
        assertTrue(DrivetrainConstants.MAX_ROTATION_SPEED > 0.0);
        assertEquals(10.0, DrivetrainConstants.AUTO_SHOOT_TOLERANCE.in(Degrees));
    }
}
