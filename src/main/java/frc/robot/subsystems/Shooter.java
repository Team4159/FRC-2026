package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    private final TalonFX hoodMotor, feederMotor, motorOne, motorTwo, motorThree, motorFour;
    private final CANcoder canCoder;

    private final PositionVoltage hoodPositionVoltage;
    private final VelocityDutyCycle shooterVelocityDutyCycle;

    public Shooter() {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCoderConfig.MagnetSensor.withMagnetOffset(Rotations.of(0));
    
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = ShooterConstants.kHoodP;  
        hoodConfig.Slot0.kI = ShooterConstants.kHoodI;
        hoodConfig.Slot0.kD = ShooterConstants.kHoodD;

        hoodConfig.Feedback.FeedbackRemoteSensorID = ShooterConstants.HoodId;
        hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        hoodConfig.Feedback.SensorToMechanismRatio = ShooterConstants.kSensorToMechanismRatio;
        hoodConfig.Feedback.RotorToSensorRatio = ShooterConstants.kMotorToSensorRatio;

        Slot0Configs shooterConfig = new Slot0Configs();
        shooterConfig.kP = ShooterConstants.kP;
        shooterConfig.kI = ShooterConstants.kI;
        shooterConfig.kD = ShooterConstants.kD;

        hoodMotor = new TalonFX(ShooterConstants.HoodId);
        feederMotor = new TalonFX(FeederConstants.FeederID);
        motorOne = new TalonFX(ShooterConstants.ShooterIDOne);
        motorTwo = new TalonFX(ShooterConstants.ShooterIDTwo);
        motorThree = new TalonFX(ShooterConstants.ShooterIDThree);
        motorFour = new TalonFX(ShooterConstants.ShooterIDFour);
        canCoder = new CANcoder(ShooterConstants.kHoodEncoderID);
        
        motorOne.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        motorTwo.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        
        canCoder.getConfigurator().apply(canCoderConfig);
        hoodMotor.getConfigurator().apply(hoodConfig);
        motorOne.getConfigurator().apply(shooterConfig);
        motorTwo.getConfigurator().apply(shooterConfig);
        motorThree.getConfigurator().apply(shooterConfig);
        motorFour.getConfigurator().apply(shooterConfig);
        
        hoodPositionVoltage = new PositionVoltage(0);
        shooterVelocityDutyCycle = new VelocityDutyCycle(0);
    }
    // set speed
    public void setSpeed(double speed) {
        shooterVelocityDutyCycle.withVelocity(speed);
        motorOne.setControl(shooterVelocityDutyCycle);
        motorTwo.setControl(shooterVelocityDutyCycle);
        motorThree.setControl(shooterVelocityDutyCycle);
        motorFour.setControl(shooterVelocityDutyCycle);
    }

    /** @return the estimated initial speed of the ball after being shot from the shooter */
    public double getFuelSpeed(){
        double motorOmega = 
            motorOne.getVelocity().getValue().in(RadiansPerSecond)
          + motorTwo.getVelocity().getValue().in(RadiansPerSecond)
          + motorThree.getVelocity().getValue().in(RadiansPerSecond)
          + motorFour.getVelocity().getValue().in(RadiansPerSecond);

        double shooterOmega = motorOmega * ShooterConstants.ratio;

        double wheelTangentialSpeed = shooterOmega * ShooterConstants.kShooterWheelRadius.in(Meters);
        double rollerTangentialSpeed = shooterOmega * ShooterConstants.kShooterRollerRadius.in(Meters);

        return ShooterConstants.kShooterEfficiency * (wheelTangentialSpeed + rollerTangentialSpeed)/2;
    }

    public void test(){
        System.out.println("test");
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