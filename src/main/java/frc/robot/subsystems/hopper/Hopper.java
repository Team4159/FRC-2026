package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperConstants.HopperSetpoint;

public class Hopper extends SubsystemBase {

    public Hopper() {}

    public void setDutyCycle(double dutyCycle) {
        HopperConstants.MOTOR.set(dutyCycle);
    }

    public void setDutyCycle(HopperSetpoint setpoint) {
        setDutyCycle(setpoint.dutyCycle);
    }

    public void stop() {
        HopperConstants.MOTOR.stopMotor();
    }

    public class ChangeSetpoint extends Command {

        private HopperSetpoint hopperSetpoint;

        public ChangeSetpoint(HopperSetpoint hopperSetpoint) {
            this.hopperSetpoint = hopperSetpoint;
            // addRequirements(Hopper.this);
        }

        @Override
        public void initialize() {
            Hopper.this.setDutyCycle(hopperSetpoint);
        }

        @Override
        public void end(boolean interrupt) {
            Hopper.this.stop();
        }
    }
}
