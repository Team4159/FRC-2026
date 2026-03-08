package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    private final TalonFX hoodMotor, feederMotor, leftBottomShooterMotor, leftTopShooterMotor, rightTopShooterMotor, rightBottomShooterMotor;
    private final CANcoder canCoder;

    private final MotionMagicVoltage hoodMotionMagic;
    private final VelocityVoltage shooterVelocityVoltage;

    private double manualAngle = 5;

    public Shooter() {
        hoodMotor = new TalonFX(ShooterConstants.HoodId);
        canCoder = new CANcoder(ShooterConstants.kHoodEncoderID);
        feederMotor = new TalonFX(FeederConstants.FeederID);
        leftBottomShooterMotor = new TalonFX(ShooterConstants.ShooterIDLeftBottom);
        leftTopShooterMotor = new TalonFX(ShooterConstants.ShooterIDLeftTop);
        rightTopShooterMotor = new TalonFX(ShooterConstants.ShooterIDRightTop);
        rightBottomShooterMotor = new TalonFX(ShooterConstants.ShooterIDRightBottom);
        
        leftBottomShooterMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        leftTopShooterMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        
        canCoder.getConfigurator().apply(ShooterConstants.canCoderConfig);
        hoodMotor.getConfigurator().apply(ShooterConstants.hoodConfig);
        leftBottomShooterMotor.getConfigurator().apply(ShooterConstants.leftShooterMotorsConfig);
        leftTopShooterMotor.getConfigurator().apply(ShooterConstants.leftShooterMotorsConfig);
        rightTopShooterMotor.getConfigurator().apply(ShooterConstants.rightShooterMotorsConfig);
        rightBottomShooterMotor.getConfigurator().apply(ShooterConstants.rightShooterMotorsConfig);

        shooterVelocityVoltage = new VelocityVoltage(0);
        hoodMotionMagic = new MotionMagicVoltage(0);
    }
    // set speed
    public void setSpeed(AngularVelocity speed) {
        shooterVelocityVoltage.withVelocity(speed.in(RotationsPerSecond));
        leftBottomShooterMotor.setControl(shooterVelocityVoltage);
        leftTopShooterMotor.setControl(shooterVelocityVoltage);
        rightTopShooterMotor.setControl(shooterVelocityVoltage);
        rightBottomShooterMotor.setControl(shooterVelocityVoltage);
    }

    /** @return the estimated initial speed of the ball after being shot from the shooter in m/s*/
    public double getFuelSpeed(){
        double motorOmega = getShooterVelocity().in(RadiansPerSecond);

        double shooterOmega = motorOmega * ShooterConstants.ratio;

        double wheelTangentialSpeed = shooterOmega * ShooterConstants.kShooterWheelRadius.in(Meters);
        double rollerTangentialSpeed = shooterOmega * ShooterConstants.kShooterRollerRadius.in(Meters);

        return ShooterConstants.kShooterEfficiency * (wheelTangentialSpeed + rollerTangentialSpeed)/2;
    }

    public AngularVelocity getShooterVelocity(){
        return  
            RadiansPerSecond.of((leftBottomShooterMotor.getVelocity().getValue().in(RadiansPerSecond)
          + leftTopShooterMotor.getVelocity().getValue().in(RadiansPerSecond)
          + rightTopShooterMotor.getVelocity().getValue().in(RadiansPerSecond)
          + rightBottomShooterMotor.getVelocity().getValue().in(RadiansPerSecond))/4);
    }

    public boolean isAtSpeed(){
        return getShooterVelocity().isNear(ShooterConstants.shooterAngularVelocity, ShooterConstants.kShooterVelocityTolerance);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("hood position", Units.rotationsToDegrees(hoodMotor.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("manual hood target", manualAngle);
        SmartDashboard.putNumber("shooter velocity", leftBottomShooterMotor.getVelocity().getValue().in(RPM));
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
    public void adjustHood(Angle angle) {
        //double angle = Math.atan((15 + Math.sqrt(Math.pow(distance, 2)-4*(gravity * Math.pow(distance, 2)) / (2 * Math.pow(fps, 2))*((gravity * Math.pow(distance, 2)) / (2 * Math.pow(fps, 2)) + height))) / (2*(gravity * Math.pow(distance, 2)) / (2 * Math.pow(fps, 2)))) * 180/Math.PI;
        hoodMotor.setControl(hoodMotionMagic.withPosition(angle));    
    }

    public void adjustTrajectoryAngle(Angle trajectoryAngle) {
        adjustHood(Degrees.of(90).minus(trajectoryAngle));
    }

    public void manualHood(double adjustment){
        double targetAngle = Math.max(5, Math.min(manualAngle + adjustment, 45));
        manualAngle = targetAngle;
        adjustHood(Degrees.of(targetAngle));
    }

    public void stopShooter(){
        leftBottomShooterMotor.stopMotor();
        leftTopShooterMotor.stopMotor();
        rightTopShooterMotor.stopMotor();
        rightBottomShooterMotor.stopMotor();
    }
    
    // public class ShooterCommand extends Command{
    //     private double velocity;
    //     private Angle angle;
        
    //     public ShooterCommand(double velocity, Angle angle){
    //         this.velocity=velocity;
    //         this.angle=angle;
            
    //         addRequirements(Shooter.this);
    //     }
        
    //     @Override
    //     public void initialize() {
    //         Shooter.this.adjustHood(angle);
    //         Shooter.this.setSpeed(velocity);
    //     }

    //     @Override
    //     public void end(boolean interrupt) {
    //         Shooter.this.setSpeed(0);
    //     }
    // }

    public class ChangeVelocity extends Command{
        private AngularVelocity velocity;
        
        public ChangeVelocity(AngularVelocity velocity){
            this.velocity = velocity;
            addRequirements(Shooter.this);
        }

        @Override
        public void initialize(){
            setSpeed(velocity);
        }

        @Override
        public void end(boolean interrupted){
            stopShooter();
        }
    }

    public class ChangeState extends Command{
        private FeederState feederState;
        
        public ChangeState(FeederState feederState){
            this.feederState = feederState;
            //addRequirements(Shooter.this);
        }

        @Override
        public void initialize(){
            Shooter.this.setFeederSpeed(feederState.percentage);
        }
        
        @Override
        public void end(boolean interrupted){
            stopFeeder();
        }
    }
}