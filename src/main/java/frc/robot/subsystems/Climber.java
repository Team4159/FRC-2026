package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.HoodConstants;

public class Climber extends SubsystemBase{
    private TalonFX climber1;
    private TalonFX climber2;
    
    private final VelocityVoltage climberVelocityVoltage;

    public Climber(){
        Slot0Configs climberConfig1 = new Slot0Configs();
        climberConfig1.kP = Constants.ClimberConstants.CkP;
        climberConfig1.kI = Constants.ClimberConstants.CkI;
        climberConfig1.kD = Constants.ClimberConstants.CkD;

        Slot0Configs climberConfig2 = new Slot0Configs();
        climberConfig2.kP = Constants.ClimberConstants.CkP;
        climberConfig2.kI = Constants.ClimberConstants.CkI;
        climberConfig2.kD = Constants.ClimberConstants.CkD;

        climber1 = new TalonFX(Constants.ClimberConstants.idClimberOne);
        climber2 = new TalonFX(Constants.ClimberConstants.idClimberTwo);

        climber1.getConfigurator().apply(climberConfig1);
        climber2.getConfigurator().apply(climberConfig2);

        climberVelocityVoltage = new VelocityVoltage(0);

    }

    public void setSpeed(double climberSpeed){
        climberVelocityVoltage.withVelocity(climberSpeed);
        climber1.setControl(climberVelocityVoltage);
        climber2.setControl(climberVelocityVoltage);
    }

    public void stopClimber(){
        climber1.setControl(climberVelocityVoltage.withVelocity(0));
        climber2.setControl(climberVelocityVoltage.withVelocity(0));
    }

}