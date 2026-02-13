package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants.HopperState;

public class Hopper extends SubsystemBase{
    private TalonFX hopperMotor;

    public Hopper(){
        hopperMotor = new TalonFX(Constants.HopperConstants.HopperId);
    }
    
    public void setHopperSpeed(double speed){
        hopperMotor.set(speed);
    }

    public void stopHopper(){
        hopperMotor.stopMotor();
    }

    public class ChangeHopperState extends Command{
        private HopperState hopperState;
        
        public ChangeHopperState(HopperState hopperState) {
            this.hopperState = hopperState;
            addRequirements(Hopper.this);
        }

        @Override
        public void initialize(){
            setHopperSpeed(hopperState.percentage);
        }

        @Override
        public void end(boolean interrupt){
            stopHopper();
        }
    }
}