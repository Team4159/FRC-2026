package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.shooter.ShooterConstants.ShooterSetpoint;
import org.junit.jupiter.api.Test;

class ShooterSetpointTest {

    @Test
    void setpointsExposeExpectedFlywheelVelocitiesAndOptionalPitches() {
        assertEquals(0.0, ShooterSetpoint.RESTING.angularVelocity.in(RPM));
        assertEquals(1000.0, ShooterSetpoint.REV.angularVelocity.in(RPM));
        assertEquals(2000.0, ShooterSetpoint.LOB.angularVelocity.in(RPM));
        assertEquals(2500.0, ShooterSetpoint.FROM_HUB.angularVelocity.in(RPM));
        assertTrue(ShooterSetpoint.FROM_HUB.pitch.isPresent());
        assertEquals(75.0, ShooterSetpoint.FROM_HUB.pitch.get().in(Degrees));
        assertEquals(70.0, ShooterSetpoint.FROM_TOWER.pitch.get().in(Degrees));
        assertFalse(ShooterSetpoint.REV.pitch.isPresent());
    }
}
