package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.HopperConstants.HopperState;

public class Hopper extends SubsystemBase{
    private TalonFX hopperMotor;

    public Hopper(){
        hopperMotor = new TalonFX(HopperConstants.HopperId);
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(20)).withSupplyCurrentLimitEnable(true);
        hopperMotor.getConfigurator().apply(currentLimits);
    }
    
    public void setHopperSpeed(double speed){
        hopperMotor.set(speed);
    }

    public void stopHopper(){
        hopperMotor.set(0);
    }

    public class ChangeState extends Command{
        private HopperState hopperState;
        
        public ChangeState(HopperState hopperState) {
            this.hopperState = hopperState;
            // addRequirements(Hopper.this);
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