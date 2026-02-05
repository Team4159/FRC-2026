package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants.ClimberState;

public class Climber extends SubsystemBase{
    private TalonFX climbMotorOne;
    private TalonFX climbMotorTwo;
    
    private final VelocityVoltage climberVelocityVoltage;

    public Climber(){
        Slot0Configs climberConfigOne = new Slot0Configs();
        climberConfigOne.kP = Constants.ClimberConstants.kP;
        climberConfigOne.kI = Constants.ClimberConstants.kI;
        climberConfigOne.kD = Constants.ClimberConstants.kD;

        Slot0Configs climberConfigTwo = new Slot0Configs();
        climberConfigTwo.kP = Constants.ClimberConstants.kP;
        climberConfigTwo.kI = Constants.ClimberConstants.kI;
        climberConfigTwo.kD = Constants.ClimberConstants.kD;

        climbMotorOne = new TalonFX(Constants.ClimberConstants.idClimberOne);
        climbMotorTwo = new TalonFX(Constants.ClimberConstants.idClimberTwo);

        climbMotorOne.getConfigurator().apply(climberConfigOne);
        climbMotorTwo.getConfigurator().apply(climberConfigTwo);

        climberVelocityVoltage = new VelocityVoltage(0);

    }

    public void setClimberSpeed(double climberSpeed){
        climberVelocityVoltage.withVelocity(climberSpeed);
        climbMotorOne.setControl(climberVelocityVoltage);
        climbMotorTwo.setControl(climberVelocityVoltage);
    }

    public void stopClimber(){
        climbMotorOne.setControl(climberVelocityVoltage.withVelocity(0));
        climbMotorTwo.setControl(climberVelocityVoltage.withVelocity(0));
    }

    public class ChangeState extends Command{
        private ClimberState climberState;
        
        public ChangeState(ClimberState climberState){
            this.climberState = climberState;
            addRequirements(Climber.this);
        }

        @Override
        public void initialize(){
            Climber.this.setClimberSpeed(climberState.percentage);
        }

        @Override
        public void end(boolean interrupt){
            Climber.this.stopClimber();
        }
    }
}