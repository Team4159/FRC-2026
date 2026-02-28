package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    private TalonFX hoodMotor, feederMotor, motorOne, motorTwo, motorThree, motorFour;
    private CANcoder canCoder;

    private final PositionVoltage hoodPositionVoltage;
    private final VelocityVoltage shooterVelocityVoltage;

    private final ArrayList<Double> angles;
    private final ArrayList<Double> distances;

    private Angle setPoint;

    public Shooter() {
        angles = new ArrayList<Double>();
        distances = new ArrayList<Double>();

        setPoint = Degrees.of(85);

        // CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        // canCoderConfig.MagnetSensor.MagnetOffset = 0.10 *3;
        // canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        Slot0Configs hoodConfig = new Slot0Configs();
        TalonFXConfiguration hoodFeedbackConfig = new TalonFXConfiguration();

        hoodConfig.kP = ShooterConstants.kPHood;  
        hoodConfig.kI = ShooterConstants.kIHood;
        hoodConfig.kD = ShooterConstants.kDHood;

        //external cancoder
        hoodFeedbackConfig.Feedback.FeedbackRemoteSensorID = ShooterConstants.kHoodCANCoderID;
        hoodFeedbackConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        hoodFeedbackConfig.Feedback.SensorToMechanismRatio = ShooterConstants.kHoodRatio;

        Slot0Configs shooterConfig = new Slot0Configs();
        shooterConfig.kP = ShooterConstants.kPShooter;
        shooterConfig.kI = ShooterConstants.kIShooter;
        shooterConfig.kD = ShooterConstants.kDShooter;

        hoodMotor = new TalonFX(ShooterConstants.hoodID);
        feederMotor = new TalonFX(FeederConstants.FeederID);
        motorOne = new TalonFX(ShooterConstants.ShooterIDOne);
        motorTwo = new TalonFX(ShooterConstants.ShooterIDTwo);
        motorThree = new TalonFX(ShooterConstants.ShooterIDThree);
        motorFour = new TalonFX(ShooterConstants.ShooterIDFour);
        //canCoder = new CANcoder(ShooterConstants.kHoodCANCoderID);

        //canCoder.getConfigurator().apply(canCoderConfig);
        
        motorThree.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        motorFour.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        
        hoodMotor.getConfigurator().apply(hoodFeedbackConfig);
        hoodMotor.getConfigurator().apply(hoodConfig);
        motorOne.getConfigurator().apply(shooterConfig);
        motorTwo.getConfigurator().apply(shooterConfig);
        motorThree.getConfigurator().apply(shooterConfig);
        motorFour.getConfigurator().apply(shooterConfig);
        
        hoodPositionVoltage = new PositionVoltage(0).withSlot(0);
        shooterVelocityVoltage = new VelocityVoltage(0);
    }
    // set speed
    /** @param speed the desired speed to be set */
    public void setSpeed(AngularVelocity angularVelocity) {
        shooterVelocityVoltage.withVelocity(angularVelocity);
        SmartDashboard.putNumber("shooterdesiredvelocity", angularVelocity.in(RadiansPerSecond));
        SmartDashboard.putNumber("feed forward shooter", shooterVelocityVoltage.FeedForward);
        motorOne.setControl(shooterVelocityVoltage.withVelocity(angularVelocity));
        motorTwo.setControl(shooterVelocityVoltage.withVelocity(angularVelocity));
        motorThree.setControl(shooterVelocityVoltage.withVelocity(angularVelocity));
        motorFour.setControl(shooterVelocityVoltage.withVelocity(angularVelocity));
    }

    public void stopShooter(){
        motorOne.set(0);
        motorTwo.set(0);
        motorThree.set(0);
        motorFour.set(0);
    }

    /** units radians */
    public double getHoodAngle(){
        return Units.rotationsToRadians(hoodMotor.getPosition().getValueAsDouble());
    }

    public void incrementSetpoint(Angle angle){
        System.out.println("incrementSetpoint");
        setPoint = setPoint.plus(angle);
        if(setPoint.in(Degrees) > 85) setPoint = Degrees.of(85);
        if(setPoint.in(Degrees) < 45) setPoint = Degrees.of(45);
    }

    @Override
    public void periodic(){
        System.out.println(hoodMotor.getConfigurator());
        SmartDashboard.putNumber("shooter velocity", motorOne.getVelocity().getValue().in(RPM));
        SmartDashboard.putNumber("hood position encoder", hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("hood angle converted", Units.radiansToDegrees(getHoodAngle()));
        SmartDashboard.putNumber("hood PID output", hoodMotor.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("setpoint hood", setPoint.in(Degrees));
        double feedForward = -Math.sin(getHoodAngle()) * ShooterConstants.kFeedForward;
        //Angle convertedAngle = setPoint.plus(ShooterConstants.angleOffset);
        //hoodMotor.setControl(hoodPositionVoltage.withPosition(setPoint).withFeedForward(feedForward));
    }

    /** @return the estimated initial speed of the ball after being shot from the shooter */
    public double getFuelSpeed(){
        double motorOmega = 
            (motorOne.getVelocity().getValue().in(RadiansPerSecond)
          + motorTwo.getVelocity().getValue().in(RadiansPerSecond)
          + motorThree.getVelocity().getValue().in(RadiansPerSecond)
          + motorFour.getVelocity().getValue().in(RadiansPerSecond))/4;

        double shooterOmega = motorOmega * ShooterConstants.ratio;

        double wheelTangentialSpeed = shooterOmega * ShooterConstants.kShooterWheelRadius.in(Meters);
        double rollerTangentialSpeed = shooterOmega * ShooterConstants.kShooterRollerRadius.in(Meters);

        return ShooterConstants.kShooterEfficiency * (wheelTangentialSpeed + rollerTangentialSpeed)/2;
    }

    public AngularVelocity getShooterVelocity(){
        return  
            RadiansPerSecond.of((motorOne.getVelocity().getValue().in(RadiansPerSecond)
          + motorTwo.getVelocity().getValue().in(RadiansPerSecond)
          + motorThree.getVelocity().getValue().in(RadiansPerSecond)
          + motorFour.getVelocity().getValue().in(RadiansPerSecond))/4);
    }

    public void test(){
        System.out.println("test");
    }

    public void logData(double distance){
        angles.add(setPoint.in(Degrees));
        distances.add(distance);

        SmartDashboard.putNumberArray("angles", toDoubleArray(angles));
        SmartDashboard.putNumberArray("distance", toDoubleArray(distances));
    }

    private double[] toDoubleArray(ArrayList<Double> list){
        double[] array = new double[list.size()];
        for(int i = 0; i < list.size(); i++){
            array[i] = list.get(i);
        }
        return array;
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
        //convert to angle of shooter from horizontal
        setPoint = angle;
        
        //hoodMotor.set(0.01);
        
    }
    
    public class ChangeVelocity extends Command{
        private AngularVelocity angularVelocity;
        
        public ChangeVelocity(AngularVelocity angularVelocity){
            this.angularVelocity= angularVelocity;
            addRequirements(Shooter.this);
        }
        
        @Override
        public void initialize() {
            if(angularVelocity.in(RadiansPerSecond) == 0){
                Shooter.this.stopShooter();
            }
            else{
                Shooter.this.setSpeed(angularVelocity);
            }
        }

        @Override
        public void end(boolean interrupt) {
            Shooter.this.stopShooter();
        }
    }

    public class ChangeHood extends Command{
        private Angle angle;

        public ChangeHood(Angle angle){
            addRequirements(Shooter.this);
        }
        
        @Override
        public void initialize() {
            if(angle.in(Degrees) > 85) angle = Degrees.of(85);
            if(angle.in(Degrees) < 45) angle = Degrees.of(45);
            adjustHood(angle);
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
            Shooter.this.setFeederSpeed(FeederState.STOP.percentage);
        }
        
    }
}