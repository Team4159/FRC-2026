package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperConstants.HopperState;

public class Hopper extends SubsystemBase {

    private final TalonFX hopperMotor;

    public Hopper() {
        hopperMotor = new TalonFX(HopperConstants.MOTOR_ID);
        hopperMotor.getConfigurator().apply(HopperConstants.MOTOR_CONFIGURATION);
    }

    public void setDutyCycle(double dutyCycle) {
        hopperMotor.set(dutyCycle);
    }

    public void stop() {
        hopperMotor.stopMotor();
    }

    public class ChangeState extends Command {

        private HopperState hopperState;

        public ChangeState(HopperState hopperState) {
            this.hopperState = hopperState;
            // addRequirements(Hopper.this);
        }

        @Override
        public void initialize() {
            Hopper.this.setDutyCycle(hopperState.dutyCycle);
        }

        @Override
        public void end(boolean interrupt) {
            Hopper.this.stop();
        }
    }
}
