package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FeederConstants {

    public static final int MOTOR_ID = 20;

    public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration() {
        {
            CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(20.0));
            MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        }
    };

    public static enum FeederState {
        FEED(1.0),
        UNJAM(-1.0),
        STOP(0.0);

        public final double dutyCycle;

        private FeederState(double dutyCycle) {
            this.dutyCycle = dutyCycle;
        }
    }
}
