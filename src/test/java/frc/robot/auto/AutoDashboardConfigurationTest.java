package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Commands;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class AutoDashboardConfigurationTest {

    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void exposesConfiguredDefaultsBeforeASelectionIsMade() {
        assertEquals(AutoConstants.START_DELAY_DEFAULT, AutoDashboardConfiguration.startDelayChooser().getSelected());
        assertEquals(AutoConstants.SHOOT_TIME_DEFAULT, AutoDashboardConfiguration.shootTimeChooser().getSelected());
        assertEquals("None", AutoDashboardConfiguration.sideChooser().getSelected());
        assertEquals("None", AutoDashboardConfiguration.intakeChooser().getSelected());
        assertEquals("None", AutoDashboardConfiguration.shootChooser().getSelected());
    }

    @Test
    void publishesEveryElasticAutoControlToSmartDashboard() {
        AutoDashboardConfiguration.publish(
            AutoDashboardConfiguration.startDelayChooser(),
            AutoDashboardConfiguration.shootTimeChooser(),
            AutoDashboardConfiguration.sideChooser(),
            AutoDashboardConfiguration.intakeChooser(),
            AutoDashboardConfiguration.shootChooser(),
            AutoDashboardConfiguration.intakeChooser(),
            AutoDashboardConfiguration.shootChooser(),
            Commands.none(),
            new Field2d()
        );

        NetworkTable dashboard = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        assertArrayEquals(
            new String[] { "Left", "Right", "Mid", "None" },
            dashboard
                .getSubTable("Auto/Side")
                .getEntry("options")
                .getStringArray(new String[0])
        );
        assertEquals("None", dashboard.getSubTable("Auto/Side").getEntry("default").getString(""));
        assertTrue(dashboard.containsKey("Auto/Generate/.type"));
        assertTrue(dashboard.containsKey("Auto/Generated Routine Display/.type"));
        assertTrue(dashboard.containsKey("Auto/Intake 1/options"));
        assertTrue(dashboard.containsKey("Auto/Shoot 1/options"));
    }
}
