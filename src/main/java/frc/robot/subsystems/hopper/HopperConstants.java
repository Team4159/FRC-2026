package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.Distance;

public class HopperConstants {

    public static final int MOTOR_ID = 30;

    public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration() {
        {
            CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(20.0));
        }
    };

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
