package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.PeripheralConstants.MotorId;

public class HopperConstants {

    public static enum HopperSetpoint {
        FEED(1.0),
        REVERSE(-1.0),
        STOP(0.0);

        public final double dutyCycle;

        private HopperSetpoint(double dutyCycle) {
            this.dutyCycle = dutyCycle;
        }
    }

    public static final TalonFX MOTOR = new TalonFX(MotorId.HOPPER_FEEDER.id);

    public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration() {
        {
            CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(20.0));
        }
    };

    public static final Distance HOPPER_EXTENT = Inches.of(12.0);

    static {
        MOTOR.getConfigurator().apply(MOTOR_CONFIGURATION);
    }
}
