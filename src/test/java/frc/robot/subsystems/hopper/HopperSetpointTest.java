package frc.robot.subsystems.hopper;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.hopper.HopperConstants.HopperSetpoint;
import org.junit.jupiter.api.Test;

class HopperSetpointTest {

    @Test
    void setpointsProvideForwardReverseAndStopOutputs() {
        assertTrue(HopperSetpoint.FEED.dutyCycle > 0.0);
        assertTrue(HopperSetpoint.REVERSE.dutyCycle < 0.0);
        assertEquals(HopperSetpoint.STOP.dutyCycle, 0.0);
    }
}
