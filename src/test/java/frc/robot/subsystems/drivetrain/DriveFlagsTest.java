package frc.robot.subsystems.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.operator.OperatorConstants.DriveFlag;
import org.junit.jupiter.api.Test;

class DriveFlagsTest {

    @Test
    void initializesEveryFlagToItsConfiguredDefault() {
        DriveFlags flags = new DriveFlags();

        assertFalse(flags.getValue(DriveFlag.SLOW_MODE));
        assertTrue(flags.getValue(DriveFlag.DRIVE_ASSIST));
        assertTrue(flags.getValue(DriveFlag.AUTO_BRAKE));
        assertFalse(flags.getValue(DriveFlag.INTAKE_ASSIST));
        assertFalse(flags.getValue(DriveFlag.ALIGN_MODE));
    }

    @Test
    void resetRestoresEachDefaultWithoutChangingOtherFlags() {
        DriveFlags flags = new DriveFlags();
        flags.setValue(DriveFlag.SLOW_MODE, true);
        flags.setValue(DriveFlag.DRIVE_ASSIST, false);
        flags.setValue(DriveFlag.AUTO_BRAKE, false);

        flags.reset();

        for (DriveFlag flag : DriveFlag.values()) {
            assertEquals(flags.getDefaultValue(flag), flags.getValue(flag), flag.name());
        }
    }
}
