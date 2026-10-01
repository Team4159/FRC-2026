package frc.robot.subsystems.hopper;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.hopper.HopperConstants.HopperSetpoint;
import org.junit.jupiter.api.Test;

class HopperSetpointTest {

    @Test
    void setpointsProvideForwardReverseAndStopOutputs() {
        assertEquals(1.0, HopperSetpoint.FEED.dutyCycle);
        assertEquals(-1.0, HopperSetpoint.REVERSE.dutyCycle);
        assertEquals(0.0, HopperSetpoint.STOP.dutyCycle);
    }
}
