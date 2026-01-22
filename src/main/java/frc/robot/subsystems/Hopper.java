package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase{
    private TalonFX hopperMotor;

    public Hopper(){
        hopperMotor = new TalonFX(Constants.HopperConstants.HopperId);
    }
    
    public void setHopperSpeed(double speed){
        hopperMotor.set(speed);
    }

    public void stop(){
        hopperMotor.set(0);
    }
}