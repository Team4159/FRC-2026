package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    private TalonFX hoodMotor, feederMotor, motorOne, motorTwo, motorThree;

    private final PositionVoltage hoodPositionVoltage;
    private final VelocityVoltage shooterVelocityVoltage;

    public Shooter() {
        Slot0Configs hoodConfig = new Slot0Configs();
        hoodConfig.kP = HoodConstants.kP;
        hoodConfig.kI = HoodConstants.kI;
        hoodConfig.kD = HoodConstants.kD;

        Slot0Configs shooterConfig = new Slot0Configs();
        shooterConfig.kP = ShooterConstants.kP;
        shooterConfig.kI = ShooterConstants.kI;
        shooterConfig.kD = ShooterConstants.kD;


        hoodMotor = new TalonFX(HoodConstants.HoodId);
        feederMotor = new TalonFX(FeederConstants.FeederID);
        motorOne = new TalonFX(ShooterConstants.ShooterIDOne);
        motorTwo = new TalonFX(ShooterConstants.ShooterIDTwo);
        motorThree = new TalonFX(ShooterConstants.ShooterIDThree);
        
        motorOne.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        
        hoodMotor.getConfigurator().apply(hoodConfig);
        motorOne.getConfigurator().apply(shooterConfig);
        motorTwo.getConfigurator().apply(shooterConfig);
        motorThree.getConfigurator().apply(shooterConfig);
        
        hoodPositionVoltage = new PositionVoltage(0);
        shooterVelocityVoltage = new VelocityVoltage(0);
    }
    // set speed
    public void setSpeed(double speed) {
        shooterVelocityVoltage.withVelocity(speed);
        motorOne.setControl(shooterVelocityVoltage);
        motorTwo.setControl(shooterVelocityVoltage);
        motorThree.setControl(shooterVelocityVoltage);
    }
    public void setFeederSpeed(double speed){
        feederMotor.set(speed);
    }
    public void stopFeeder(){
        feederMotor.set(0);
    }
    // adjust hood
    public void adjustHood(double angle) {
        //double angle = Math.atan((15 + Math.sqrt(Math.pow(distance, 2)-4*(gravity * Math.pow(distance, 2)) / (2 * Math.pow(fps, 2))*((gravity * Math.pow(distance, 2)) / (2 * Math.pow(fps, 2)) + height))) / (2*(gravity * Math.pow(distance, 2)) / (2 * Math.pow(fps, 2)))) * 180/Math.PI;
        hoodMotor.setControl(hoodPositionVoltage.withPosition(angle));    
    }
    
    public class ShooterCommand extends Command{
        private double velocity;
        private double angle;
        
        public ShooterCommand(double velocity, double angle){
            this.velocity=velocity;
            this.angle=angle;
            
            addRequirements(Shooter.this);
        }
        
        @Override
        public void initialize() {
            Shooter.this.adjustHood(angle);
            Shooter.this.setSpeed(velocity);
        }

        @Override
        public void end(boolean interrupt) {
            Shooter.this.setSpeed(0);
        }
    }

    public class ChangeState extends Command{
        private FeederState feederState;
        
        public ChangeState(FeederState feederState){
            this.feederState = feederState;
            addRequirements(Shooter.this);
        }

        @Override
        public void initialize(){
            Shooter.this.setFeederSpeed(feederState.percentage);
        }
        
    }
}
