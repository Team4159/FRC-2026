package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public class HopperConstants {

    public static final int HOPPER_MOTOR_ID = 30;
    public static final Distance HOPPER_EXTENT = Inches.of(12.0);

    public static enum HopperState {
        FEED(1.0),
        REVERSE(-1.0),
        STOP(0.0);

        public final double dutyCycle;

        private HopperState(double dutyCycle) {
            this.dutyCycle = dutyCycle;
        }
    }
}
