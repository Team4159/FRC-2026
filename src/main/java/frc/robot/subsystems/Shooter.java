package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    //all TalonFX motors on the shooter (hood and feeder are X44, shooter motors are X60 but in code all TalonFX motors (Falcon, Kraken x44 and x60) all behave the same)
    private final TalonFX hoodMotor, feederMotor, leftBottomShooterMotor, leftTopShooterMotor, rightTopShooterMotor, rightBottomShooterMotor;
    //this was cooked for some reason never got a chance to figure out why so instead we just set each motor individually instead of using the leader/follower system
    //private final TalonFX leaderShooterMotor;
    //the CANCoder on the hood
    private final CANcoder hoodCanCoder;

    //Phoenix control requests
    //this allows for control systems to be run on the motor controllers (less work for roborio) because CTRE is actually good at this (unlike REV, for REV motors just use a WPILIB PID)

    //Motion Magic® is a CTRE control request that is a profiled PID controller that takes a max velocity, acceleration, and jerk into account for smoother motion control, works very well for pivots
    private final MotionMagicVoltage hoodMotionMagic;
    //velocity voltage is just a regular voltage based PID for a velocity setpoint
    private final VelocityVoltage shooterVelocityVoltage;

    //the current manual angle setpoint in degrees
    private double manualAngle = 5;

    public Shooter() {
        //initialize motors and CANCoder using the CANIDs in constants
        hoodMotor = new TalonFX(ShooterConstants.HoodId);
        hoodCanCoder = new CANcoder(ShooterConstants.kHoodEncoderID);
        feederMotor = new TalonFX(FeederConstants.FeederID);
        leftBottomShooterMotor = new TalonFX(ShooterConstants.ShooterIDLeftBottom);
        leftTopShooterMotor = new TalonFX(ShooterConstants.ShooterIDLeftTop);
        rightTopShooterMotor = new TalonFX(ShooterConstants.ShooterIDRightTop);
        rightBottomShooterMotor = new TalonFX(ShooterConstants.ShooterIDRightBottom);

        //leaderShooterMotor = leftBottomShooterMotor;
        
        //these should no longer be necessary with the other configs applied after this but i (Trevor Choy) left it in because i was worried about breaking something at comp lol
        leftBottomShooterMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        leftTopShooterMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        
        //apply the configs
        hoodCanCoder.getConfigurator().apply(ShooterConstants.canCoderConfig);
        hoodMotor.getConfigurator().apply(ShooterConstants.hoodConfig);
        leftBottomShooterMotor.getConfigurator().apply(ShooterConstants.leftShooterMotorsConfig);
        leftTopShooterMotor.getConfigurator().apply(ShooterConstants.leftShooterMotorsConfig);
        rightTopShooterMotor.getConfigurator().apply(ShooterConstants.rightShooterMotorsConfig);
        rightBottomShooterMotor.getConfigurator().apply(ShooterConstants.rightShooterMotorsConfig);

        //the feeder motor does not need much of a config so it just has its current limit config created and applied here
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(20)).withSupplyCurrentLimitEnable(true);
        feederMotor.getConfigurator().apply(currentLimits);

        //initialize the control requests(the setpoints are changed later and are currently meaningless)
        shooterVelocityVoltage = new VelocityVoltage(0);
        hoodMotionMagic = new MotionMagicVoltage(0);

        //set the hood to the resting position
        //when making commands for the shooter the hood should always be set back to resting position when done so the robot can go under the trench
        restHood();
        //set speed of the shooter wheels ot the resting velocity (makes it take less time to spin up and shoot, didn't cause any brownouts at Contra Costa but could use some more testing)
        this.setSpeed(Constants.ShooterConstants.restingAngularVelocity);

        // leftTopShooterMotor.setControl(new StrictFollower(leaderShooterMotor.getDeviceID()));
        // rightTopShooterMotor.setControl(new StrictFollower(leaderShooterMotor.getDeviceID()));
        // rightBottomShooterMotor.setControl(new StrictFollower(leaderShooterMotor.getDeviceID()));
    }
    /** @param deisredAngularVelocity the desired angular velocity of the motors */
    public void setSpeed(AngularVelocity desiredAngularVelocity) {
        //set the velocity target of the velocity voltage to the desired angular velocity
        shooterVelocityVoltage.withVelocity(desiredAngularVelocity.in(RotationsPerSecond));
        //leaderShooterMotor.setControl(shooterVelocityVoltage);
        //set the control of the motors to the velocityVoltage
        leftBottomShooterMotor.setControl(shooterVelocityVoltage.withVelocity(desiredAngularVelocity.in(RotationsPerSecond)));
        leftTopShooterMotor.setControl(shooterVelocityVoltage.withVelocity(desiredAngularVelocity.in(RotationsPerSecond)));
        rightTopShooterMotor.setControl(shooterVelocityVoltage.withVelocity(desiredAngularVelocity.in(RotationsPerSecond)));
        rightBottomShooterMotor.setControl(shooterVelocityVoltage.withVelocity(desiredAngularVelocity.in(RotationsPerSecond)));
    }

    /** @return the estimated initial speed of the ball after being shot from the shooter in m/s*/
    public double getFuelSpeed(){
        return getFuelSpeedWithCustomEfficiency(ShooterConstants.kShooterEfficiency);
    }

    /** @param efficiency the percentage of velocity transferred from the wheels to the fuel SUPPOSED to be from (0-1) but sometimes cooked things happen and you get 1.05 as your efficiency 😭 likely because of forgetting the hood roller gear ratio
     * @return the estimated initial speed of the ball after being shot from the shooter in m/s*/
    public double getFuelSpeedWithCustomEfficiency(double efficiency){
        
        //angular velocity of the motors
        double motorOmega = getShooterMotorVelocity().in(RadiansPerSecond);

        //angular velocity of the flywheels
        double shooterOmega = motorOmega * ShooterConstants.ratio;

        //tangential speed of the flywheel wheels
        double wheelTangentialSpeed = shooterOmega * ShooterConstants.kShooterWheelRadius.in(Meters);

        //tangential speed of the roller hood wheels (I just realized I forgot to account for the gear ratio lol 😭)
        double rollerTangentialSpeed = shooterOmega * ShooterConstants.kShooterRollerRadius.in(Meters);

        //return an average of the two velocities times the efficiency to get the estimated fuel velocity
        return efficiency * (wheelTangentialSpeed + rollerTangentialSpeed)/2;
    }

    /** @return the average angular velocity of the shooter motors measured from all 4 shooter motors*/
    public AngularVelocity getShooterMotorVelocity(){
            return  
            RadiansPerSecond.of((leftBottomShooterMotor.getVelocity().getValue().in(RadiansPerSecond)
          + leftTopShooterMotor.getVelocity().getValue().in(RadiansPerSecond)
          + rightTopShooterMotor.getVelocity().getValue().in(RadiansPerSecond)
          + rightBottomShooterMotor.getVelocity().getValue().in(RadiansPerSecond))/4);
    }

    /** @return true if the shooter motors are at the target velocity (within tolerance), false otherwise*/
    public boolean isAtSpeed(){
        return leftBottomShooterMotor.getClosedLoopReference().isNear(getShooterMotorVelocity().in(RotationsPerSecond), ShooterConstants.kShooterVelocityTolerance.in(RotationsPerSecond));
    }

    /** @ return true if the hood is at the right pitch within tolerance, false otherwise */
    public boolean isAtPitch() {
        //5 degrees of tolerance should be a constant instead
        return hoodMotor.getClosedLoopReference().isNear(hoodMotor.getPosition().getValueAsDouble(), Units.degreesToRotations(5));
    }

    @Override
    public void periodic(){
        //just a bunch of smartdashboard logging used for tuning
        SmartDashboard.putNumber("hood position", Units.rotationsToDegrees(hoodMotor.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("hood target position", hoodMotionMagic.Position);
        SmartDashboard.putNumber("manual hood target", manualAngle);
        SmartDashboard.putNumber("shooter velocity", leftBottomShooterMotor.getVelocity().getValue().in(RPM));

        SmartDashboard.putNumber("bottom left shooter motor current", leftBottomShooterMotor.getSupplyCurrent().getValue().in(Amps));
        SmartDashboard.putNumber("top left shooter motor current", leftTopShooterMotor.getSupplyCurrent().getValue().in(Amps));
        SmartDashboard.putNumber("bottom right shooter motor current", rightBottomShooterMotor.getSupplyCurrent().getValue().in(Amps));
        SmartDashboard.putNumber("top right shooter motor current", rightTopShooterMotor.getSupplyCurrent().getValue().in(Amps));
    }
    
    /** @param speed the percentage (-1-1) of how much power is sent to the feeder motor*/
    public void setFeederSpeed(double speed){
        feederMotor.set(speed);
    }
    /** stops the feeder */
    public void stopFeeder(){
        feederMotor.set(0);
    }
    /** @param desiredAngle set the deisred angle of the hood*/
    public void adjustHood(Angle desiredAngle) {
        //set the hood motor control to the motion magic with a desired position that is the desired angle
        hoodMotor.setControl(hoodMotionMagic.withPosition(desiredAngle));    
    }
    /** sets the desired angle of the hood to the resting angle(fits under the trench) */
    public void restHood() {
        adjustHood(ShooterConstants.kRestingAngle);
    }
    /** @param trajectoryAngle the desired launch angle of the fuel
     * adjusts the hood such to achieve the desired fuel launch angle
     */
    public void adjustTrajectoryAngle(Angle trajectoryAngle) {
        //adjusthood is in terms of shooter angle where the angle of the shooter COM with respect to the horizontal is 0, to get this from trajectory angle must get the complement of the trajectory angle
        //subtract the hood offset which is the angle between the hood COM and the final hood roller
        adjustHood(Degrees.of(90).minus(trajectoryAngle).minus(ShooterConstants.kHoodAngleOffset));
    }
    /** @param adjustment how much to adjust by in degrees */
    public void manualHood(double adjustment){
        //make sure the hood setpoint stays within its bounded range
        double targetAngle = Math.max(5, Math.min(manualAngle + adjustment, 45));
        //store the setpoint in the manualAngle member variable
        manualAngle = targetAngle;
        //adjust the hood to the manual setpoint
        adjustHood(Degrees.of(targetAngle));
    }

    /** stop all shooter motors */
    public void stopShooter(){
        leftBottomShooterMotor.stopMotor();
        leftTopShooterMotor.stopMotor();
        rightTopShooterMotor.stopMotor();
        rightBottomShooterMotor.stopMotor();
    }

    /** A command to run the shooter motors at a given velocity */
    public class ChangeVelocity extends Command{
        private AngularVelocity velocity;
        
        /** @param velocity the deisred angular velocity of the shooter
         * this command requires the shooter subsystem (meaning that on initialize it terminates any other command running that also requires the shooter command, and will get terminated if another command that requires shooter is initialized)
         */
        public ChangeVelocity(AngularVelocity velocity){
            this.velocity = velocity;
            addRequirements(Shooter.this);
        }

        @Override
        public void initialize(){
            //set the shooter target speed to the desired angular velocity
            //"Shooter.this" is not needed to use setSpeed, it can be accessed directly due to the command being a nested class of the Shooter subsystem
            setSpeed(velocity);
        }

        @Override
        public void end(boolean interrupted){
            //stop the shooter when the command ends
            stopShooter();
        }
    }

    /** ChangeState just changes the feeder state (only feeder uses enum states because the rest of the shooter has to dynamically change with auto aim rather than have discrete setpoints)
     * the name is still misleading and should be changed to ChangeFeederState to be more clear
     */
    public class ChangeState extends Command{
        private FeederState feederState;
        
        /** @param feederState the desired feeder state */
        public ChangeState(FeederState feederState){
            this.feederState = feederState;
        }

        @Override
        public void initialize(){
            //set the feeder speed to the percentage in the state object
            Shooter.this.setFeederSpeed(feederState.percentage);
        }
        
        @Override
        public void end(boolean interrupted){
            //stop the feeder when the command ends
            stopFeeder();
        }
    }
}