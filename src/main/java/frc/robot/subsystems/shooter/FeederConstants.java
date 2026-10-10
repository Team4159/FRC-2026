package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.PeripheralConstants.MotorId;

public class FeederConstants {

    public static enum FeederSetpoint {
        FEED(1.0),
        UNJAM(-1.0),
        STOP(0.0);

        public final double dutyCycle;

        private FeederSetpoint(double dutyCycle) {
            this.dutyCycle = dutyCycle;
        }
    }

    public static final TalonFX MOTOR = new TalonFX(MotorId.SHOOTER_NECK_FEEDER.id);

    public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration() {
        {
            CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(20.0));
            MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        }
    };

    static {
        MOTOR.getConfigurator().apply(MOTOR_CONFIGURATION);
    }
}
