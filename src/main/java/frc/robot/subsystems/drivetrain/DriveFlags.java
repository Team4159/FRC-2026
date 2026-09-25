package frc.robot.subsystems.drivetrain;

import frc.robot.operator.OperatorConstants.DriveFlag;
import java.util.HashMap;
import java.util.Map;

public class DriveFlags {

    private class DriveFlagValue {

        public final boolean defaultValue;
        public boolean value;

        public DriveFlagValue(boolean defaultValue) {
            this.defaultValue = defaultValue;
            this.value = defaultValue;
        }

        public void reset() {
            value = defaultValue;
        }
    }

    private final Map<DriveFlag, DriveFlagValue> driveFlags = new HashMap<DriveFlag, DriveFlagValue>();

    {
        driveFlags.put(DriveFlag.SLOW_MODE, new DriveFlagValue(false));
        driveFlags.put(DriveFlag.DRIVE_ASSIST, new DriveFlagValue(true));
        driveFlags.put(DriveFlag.AUTO_BRAKE, new DriveFlagValue(true));
        driveFlags.put(DriveFlag.INTAKE_ASSIST, new DriveFlagValue(false));
        driveFlags.put(DriveFlag.MANUAL_ALIGN, new DriveFlagValue(false));
    }

    DriveFlags() {}

    public void setValue(DriveFlag driveFlag, boolean newValue) {
        driveFlags.get(driveFlag).value = newValue;
    }

    public boolean getValue(DriveFlag driveFlag) {
        return driveFlags.get(driveFlag).value;
    }

    public boolean getDefaultValue(DriveFlag driveFlag) {
        return driveFlags.get(driveFlag).defaultValue;
    }

    public void reset() {
        driveFlags.forEach((key, value) -> value.reset());
    }
}
