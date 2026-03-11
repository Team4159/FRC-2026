package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberState;

public class Climber {

    public static Distance angleToPosition(Angle angle) {
        return Meters.of(angle.in(Rotations) * ClimberConstants.kAngleToPosition);
    }

    public static Angle positionToAngle(Distance position) {
        return Rotations.of(position.in(Meters) / ClimberConstants.kAngleToPosition);
    }

    private final TalonFX elevatorMotor = new TalonFX(15);
    private final PositionVoltage elevatorPositionVoltage = new PositionVoltage(0);
    {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration() {
            {
                MotorOutput.NeutralMode = NeutralModeValue.Brake;
                HardwareLimitSwitch.ReverseLimitRemoteSensorID = 0;
                HardwareLimitSwitch.ReverseLimitEnable = true;
                HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
                HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
                HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
                HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;
            }
        };

        elevatorMotor.getConfigurator().apply(talonFXConfiguration);
    }

    private ClimberState state = ClimberState.LOWERED;

    public Climber() {
        setState(state);
    }

    public void setPosition(Distance position) {
        elevatorPositionVoltage.withPosition(positionToAngle(position));
        elevatorMotor.setControl(elevatorPositionVoltage);
    }

    public void setState(ClimberState newState) {
        state = newState;
        setPosition(newState.position);
    }

    public class ZeroElevator extends Command {
        private boolean lift;
        private double liftStart;

        public ZeroElevator() {

        }

        @Override
        public void initialize() {
            lift = limitSwitchHit();
            liftStart = MathSharedStore.getTimestamp();
        }

        @Override
        public void execute() {
            if (lift && MathSharedStore.getTimestamp() - liftStart >= ClimberConstants.kZeroLiftDuration && !limitSwitchHit()) {
                lift = false;
            }
            if (lift) {
                // up
                elevatorMotor.set(-ClimberConstants.kZeroDutyCycle);
            } else {
                // down
                elevatorMotor.set(ClimberConstants.kZeroDutyCycle);
            }
        }

        @Override
        public boolean isFinished() {
            return limitSwitchHit();
        }

        private boolean limitSwitchHit() {
            return elevatorMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
        }
    }
}
