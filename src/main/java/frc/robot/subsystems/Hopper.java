package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase{
    private TalonFX hopperMotor;
    private PositionVoltage hopperMotorPositionVoltage;

    public Hopper(){
        hopperMotor = new TalonFX(Constants.HopperConstants.HopperId);
        hopperMotorPositionVoltage = new PositionVoltage(0);
    }
    
    public void setHopperSpeed(double speed){
        hopperMotor.set(speed);
    }
}