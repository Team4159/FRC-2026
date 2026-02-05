package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

//:::::::::       :::    ::       ::  ::::::::::  :::::::::
//::      ::     :: ::   ::       ::      ::      ::      ::
//::       ::    :: ::    ::     ::       ::      ::       ::
//::       ::   ::   ::   ::     ::       ;;      ::       ::
//::       ::   ::::::::   ::   ::        ::      ::       ::
//::       ::  ::     ::   ::   ::        ::      ::       ::
//::       ::  ::     ::    :: ::         ::      ::       ::
//::      ::  ::       ::   :: ::         ::      ::      ::  
//:::::::::   ::       ::    :::      ::::::::::  :::::::::    

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberState;

public class Climber extends SubsystemBase{
    private TalonFX climbMotorOne, climbMotorTwo;
    
    private final VelocityVoltage climberVelocityVoltage;

    public Climber(){
        Slot0Configs climberConfigOne = new Slot0Configs();
        climberConfigOne.kP = ClimberConstants.kP;
        climberConfigOne.kI = ClimberConstants.kI;
        climberConfigOne.kD = ClimberConstants.kD;

        Slot0Configs climberConfigTwo = new Slot0Configs();
        climberConfigTwo.kP = ClimberConstants.kP;
        climberConfigTwo.kI = ClimberConstants.kI;
        climberConfigTwo.kD = ClimberConstants.kD;

        climbMotorOne = new TalonFX(ClimberConstants.idClimberOne);
        climbMotorTwo = new TalonFX(ClimberConstants.idClimberTwo);

        climbMotorOne.getConfigurator().apply(climberConfigOne);
        climbMotorTwo.getConfigurator().apply(climberConfigTwo);

        climberVelocityVoltage = new VelocityVoltage(0);

    }

    public void setClimberSpeed(double speed){
        climberVelocityVoltage.withVelocity(speed);
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