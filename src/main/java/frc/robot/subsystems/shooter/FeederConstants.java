package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FeederConstants {

    public static enum FeederState {
        FEED(1.0),
        UNJAM(-1.0),
        STOP(0.0);

        public final double dutyCycle;

        private FeederState(double dutyCycle) {
            this.dutyCycle = dutyCycle;
        }
    }

    public static final TalonFX MOTOR = new TalonFX(20);

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
