package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperConstants.HopperState;

public class Hopper extends SubsystemBase {

    public Hopper() {}

    public void setDutyCycle(double dutyCycle) {
        HopperConstants.MOTOR.set(dutyCycle);
    }

    public void stop() {
        HopperConstants.MOTOR.stopMotor();
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
