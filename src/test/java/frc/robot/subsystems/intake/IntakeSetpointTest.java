package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.intake.IntakeConstants.IntakeSetpoint;
import org.junit.jupiter.api.Test;

class IntakeSetpointTest {

    @Test
    void setpointsPairExpectedPivotAnglesAndRollerOutputs() {
        assertEquals(-9.0, IntakeSetpoint.DOWN_ON.pivotAngle.in(Degrees));
        assertEquals(1.0, IntakeSetpoint.DOWN_ON.rollerDutyCycle);
        assertEquals(-1.0, IntakeSetpoint.DOWN_REVERSE.rollerDutyCycle);
        assertEquals(0.0, IntakeSetpoint.DOWN_OFF.rollerDutyCycle);
        assertEquals(120.0, IntakeSetpoint.UP_OFF.pivotAngle.in(Degrees));
        assertEquals(60.0, IntakeSetpoint.BOUNCE_UP.pivotAngle.in(Degrees));
    }
}
