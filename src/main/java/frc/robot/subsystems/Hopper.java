package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut; //read what I put in climber

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants.HopperState;

public class Hopper extends SubsystemBase{
    private TalonFX hopperMotor;
    private final DutyCycleOut percentOutput;

    public Hopper(){
        hopperMotor = new TalonFX(Constants.HopperConstants.HopperId);
        percentOutput = new DutyCycleOut(0);
    }
    
    public void setHopperSpeed(double speed){
        hopperMotor.setControl(percentOutput.withOutput(speed));
    }

    public void stopHopper(){
        hopperMotor.setControl(percentOutput.withOutput(0));
    }

    public class ChangeState extends Command{
        private HopperState hopperState;
        
        public ChangeState(HopperState hopperState) {
            this.hopperState = hopperState;
            addRequirements(Hopper.this);
        }

        @Override
        public void initialize(){
            Hopper.this.setHopperSpeed(hopperState.percentage);
        }

        @Override
        public void end(boolean interrupt){
            Hopper.this.stopHopper();
        }
    }
}