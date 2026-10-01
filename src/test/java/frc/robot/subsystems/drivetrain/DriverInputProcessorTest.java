package frc.robot.subsystems.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.operator.OperatorConstants;
import org.junit.jupiter.api.Test;

class DriverInputProcessorTest {

    @Test
    void removesInputsInsideDriverDeadband() {
        Translation2d result = DriverInputProcessor.translation(
            0.03,
            -0.04,
            OperatorConstants.PRIMARY_TRANSLATION_DEADBAND,
            OperatorConstants.PRIMARY_TRANSLATION_RADIUS,
            OperatorConstants.PRIMARY_TRANSLATION_EXPONENT
        );

        assertEquals(0.0, result.getNorm(), 1e-9);
        assertEquals(0.0, DriverInputProcessor.rotation(0.04, 0.05, 2.0), 1e-9);
    }

    @Test
    void preservesDirectionAndAppliesSquaredDriverResponse() {
        Translation2d result = DriverInputProcessor.translation(0.5, -0.5, 0.0, 1.0, 2.0);

        assertEquals(Math.sqrt(0.125), result.getX(), 1e-9);
        assertEquals(-Math.sqrt(0.125), result.getY(), 1e-9);
        assertEquals(-0.25, DriverInputProcessor.rotation(-0.5, 0.0, 2.0), 1e-9);
    }

    @Test
    void limitsDiagonalInputsToUnitMagnitude() {
        Translation2d result = DriverInputProcessor.translation(1.0, 1.0, 0.0, 1.0, 1.0);

        assertEquals(1.0, result.getNorm(), 1e-9);
        assertTrue(result.getX() > 0.0);
        assertTrue(result.getY() > 0.0);
    }
}
