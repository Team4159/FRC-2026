package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants.ClimberState;

public class Climber extends SubsystemBase{
    private TalonFX climber1;
    private TalonFX climber2;
    
    private final VelocityVoltage climberVelocityVoltage;

    public Climber(){
        Slot0Configs climberConfig1 = new Slot0Configs();
        climberConfig1.kP = Constants.ClimberConstants.kP;
        climberConfig1.kI = Constants.ClimberConstants.kI;
        climberConfig1.kD = Constants.ClimberConstants.kD;

        Slot0Configs climberConfig2 = new Slot0Configs();
        climberConfig2.kP = Constants.ClimberConstants.kP;
        climberConfig2.kI = Constants.ClimberConstants.kI;
        climberConfig2.kD = Constants.ClimberConstants.kD;

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

    public class ChangeState extends Command{
        private ClimberState climberState;
        public ChangeState(ClimberState climberState){
            this.climberState = climberState;
            addRequirements(Climber.this);
        }

        @Override
        public void initialize(){
            Climber.this.setSpeed(climberState.percentage);
        }

        @Override
        public void end(boolean interrupt){
            Climber.this.stopClimber();
        }
    }
}