package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class AutoPathNamesTest {

    @Test
    void buildsStandardAutoPathNames() {
        assertEquals("StartToFarIntake", AutoPathNames.startToIntake("FarIntake"));
        assertEquals("FarIntakeToShoot", AutoPathNames.intakeToShoot("FarIntake", "Shoot"));
        assertEquals("ShootToCloseIntake", AutoPathNames.shootToIntake("Shoot", "CloseIntake"));
    }

    @Test
    void buildsMiddleAndOutpostPathNames() {
        assertEquals("MStartToShoot", AutoPathNames.middleStartToShoot("M"));
        assertEquals("MLStartToShoot", AutoPathNames.middleStartToShoot("ML"));
        assertEquals("MStartToMROutpostIntake", AutoPathNames.outpostStartToIntake("M"));
        assertEquals("MROutpostIntakeToMRShoot", AutoPathNames.outpostIntakeToShoot());
    }
}
