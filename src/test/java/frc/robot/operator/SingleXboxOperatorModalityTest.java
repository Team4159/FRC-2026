package frc.robot.operator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SingleXboxOperatorModalityTest {

    private static final int PORT = 0;
    private SingleXboxOperatorModality modality;
    private XboxControllerSim controller;

    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        modality = new SingleXboxOperatorModality(PORT, 0.1);
        controller = new XboxControllerSim(modality.getHID());
    }

    @Test
    void mapsStickAxesToRobotMotionWithExpectedSigns() {
        controller.setLeftY(0.7);
        controller.setLeftX(-0.4);
        controller.setRightX(0.25);
        DriverStationSim.notifyNewData();

        assertEquals(-0.7, modality.translateX(), 1e-6);
        assertEquals(0.4, modality.translateY(), 1e-6);
        assertEquals(-0.25, modality.rotation(), 1e-6);
    }

    @Test
    void selectsExactlyOneShootModeFromBumperAndTrigger() {
        controller.setRightBumperButton(true);
        DriverStationSim.notifyNewData();
        assertTrue(modality.autoShoot().getAsBoolean());
        assertFalse(modality.hubShoot().getAsBoolean());
        assertFalse(modality.towerShoot().getAsBoolean());

        controller.setRightTriggerAxis(0.5);
        DriverStationSim.notifyNewData();
        assertFalse(modality.autoShoot().getAsBoolean());
        assertFalse(modality.hubShoot().getAsBoolean());
        assertTrue(modality.towerShoot().getAsBoolean());

        controller.setRightBumperButton(false);
        DriverStationSim.notifyNewData();
        assertFalse(modality.autoShoot().getAsBoolean());
        assertTrue(modality.hubShoot().getAsBoolean());
        assertFalse(modality.towerShoot().getAsBoolean());
    }

    @Test
    void mapsIntakeAndUtilityControls() {
        controller.setLeftTriggerAxis(0.3);
        controller.setLeftBumperButton(true);
        controller.setXButton(true);
        controller.setBButton(true);
        controller.setYButton(true);
        controller.setBackButton(true);
        controller.setPOV(270);
        DriverStationSim.notifyNewData();

        assertTrue(modality.intake().getAsBoolean());
        assertTrue(modality.slowMode().getAsBoolean());
        assertTrue(modality.outtake().getAsBoolean());
        assertTrue(modality.retractIntake().getAsBoolean());
        assertTrue(modality.driverAssist().getAsBoolean());
        assertTrue(modality.zero().getAsBoolean());
        assertTrue(modality.revShooter().getAsBoolean());
    }
}
