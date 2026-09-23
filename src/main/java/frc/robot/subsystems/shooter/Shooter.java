package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.FeederConstants.FeederState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterSetpoint;

public class Shooter extends SubsystemBase {

    //all TalonFX motors on the shooter (hood and feeder are X44, shooter motors are X60 but in code all TalonFX motors (Falcon, Kraken x44 and x60) all behave the same)
    private final TalonFX hoodMotor, feederMotor, leftBottomFlywheelMotor, leftTopFlywheelMotor, rightTopFlywheelMotor, rightBottomFlywheelMotor;
    //this was cooked for some reason never got a chance to figure out why so instead we just set each motor individually instead of using the leader/follower system
    //private final TalonFX leaderShooterMotor;
    //the CANCoder on the hood
    private final CANcoder hoodEncoder;

    //Phoenix control requests
    //this allows for control systems to be run on the motor controllers (less work for roborio) because CTRE is actually good at this (unlike REV, for REV motors just use a WPILIB PID)

    //Motion Magic® is a CTRE control request that is a profiled PID controller that takes a max velocity, acceleration, and jerk into account for smoother motion control, works very well for pivots
    private final MotionMagicVoltage hoodMotionMagicVoltage;
    //velocity voltage is just a regular voltage based PID for a velocity setpoint
    private final VelocityVoltage flywheelVelocityVoltage;

    //the current manual angle setpoint in degrees
    private double manualAngle = 5.0;

    private final Debouncer velocityDebouncer = new Debouncer(0.2, DebounceType.kBoth);

    public Shooter() {
        //initialize motors and CANCoder using the CANIDs in constants
        hoodMotor = new TalonFX(HoodConstants.MOTOR_ID);
        hoodEncoder = new CANcoder(HoodConstants.ENCODER_ID);
        feederMotor = new TalonFX(FeederConstants.MOTOR_ID);
        leftBottomFlywheelMotor = new TalonFX(ShooterConstants.LEFT_BOTTOM_MOTOR_ID);
        leftTopFlywheelMotor = new TalonFX(ShooterConstants.LEFT_TOP_MOTOR_ID);
        rightTopFlywheelMotor = new TalonFX(ShooterConstants.RIGHT_TOP_MOTOR_ID);
        rightBottomFlywheelMotor = new TalonFX(ShooterConstants.RIGHT_BOTTOM_MOTOR_ID);

        //apply the configs
        hoodMotor.getConfigurator().apply(HoodConstants.MOTOR_CONFIGURATION);
        hoodEncoder.getConfigurator().apply(HoodConstants.ENCODER_CONFIGURATION);
        feederMotor.getConfigurator().apply(FeederConstants.MOTOR_CONFIGURATION);
        leftBottomFlywheelMotor.getConfigurator().apply(ShooterConstants.LEFT_MOTORS_CONFIGURATION);
        leftTopFlywheelMotor.getConfigurator().apply(ShooterConstants.LEFT_MOTORS_CONFIGURATION);
        rightTopFlywheelMotor.getConfigurator().apply(ShooterConstants.RIGHT_MOTORS_CONFIGURATION);
        rightBottomFlywheelMotor.getConfigurator().apply(ShooterConstants.RIGHT_MOTORS_CONFIGURATION);

        //initialize the control requests(the setpoints are changed later and are currently meaningless)
        flywheelVelocityVoltage = new VelocityVoltage(0.0);
        hoodMotionMagicVoltage = new MotionMagicVoltage(0.0);

        //set the hood to the resting position
        //when making commands for the shooter the hood should always be set back to resting position when done so the robot can go under the trench
        restHood();
        //set velocity of the shooter wheels ot the resting velocity (makes it take less time to spin up and shoot, didn't cause any brownouts at Contra Costa but could use some more testing)
        restFlywheel();

        // leftTopShooterMotor.setControl(new StrictFollower(leaderShooterMotor.getDeviceID()));
        // rightTopShooterMotor.setControl(new StrictFollower(leaderShooterMotor.getDeviceID()));
        // rightBottomShooterMotor.setControl(new StrictFollower(leaderShooterMotor.getDeviceID()));
    }

    @Override
    public void periodic() {
        //just a bunch of smartdashboard logging used for tuning
        SmartDashboard.putNumber("hood position", Units.rotationsToDegrees(hoodMotor.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("hood target position", hoodMotionMagicVoltage.Position);
        SmartDashboard.putNumber("manual hood target", manualAngle);
        SmartDashboard.putNumber("shooter velocity", getFlywheelMotorVelocity().in(RPM));
        SmartDashboard.putNumber("shooter velocity target", flywheelVelocityVoltage.getVelocityMeasure().in(RPM));

        SmartDashboard.putNumber(
            "bottom left shooter motor current",
            leftBottomFlywheelMotor.getSupplyCurrent().getValue().in(Amps)
        );
        SmartDashboard.putNumber(
            "top left shooter motor current",
            leftTopFlywheelMotor.getSupplyCurrent().getValue().in(Amps)
        );
        SmartDashboard.putNumber(
            "bottom right shooter motor current",
            rightBottomFlywheelMotor.getSupplyCurrent().getValue().in(Amps)
        );
        SmartDashboard.putNumber(
            "top right shooter motor current",
            rightTopFlywheelMotor.getSupplyCurrent().getValue().in(Amps)
        );
    }

    /** @param deisredAngularVelocity the desired angular velocity of the motors */
    public void setFlywheelVelocity(AngularVelocity desiredAngularVelocity) {
        //set the velocity target of the velocity voltage to the desired angular velocity
        flywheelVelocityVoltage.withVelocity(desiredAngularVelocity.in(RotationsPerSecond));
        //leaderShooterMotor.setControl(shooterVelocityVoltage);
        //set the control of the motors to the velocityVoltage
        leftBottomFlywheelMotor.setControl(
            flywheelVelocityVoltage.withVelocity(desiredAngularVelocity.in(RotationsPerSecond))
        );
        leftTopFlywheelMotor.setControl(
            flywheelVelocityVoltage.withVelocity(desiredAngularVelocity.in(RotationsPerSecond))
        );
        rightTopFlywheelMotor.setControl(
            flywheelVelocityVoltage.withVelocity(desiredAngularVelocity.in(RotationsPerSecond))
        );
        rightBottomFlywheelMotor.setControl(
            flywheelVelocityVoltage.withVelocity(desiredAngularVelocity.in(RotationsPerSecond))
        );
    }

    public void setFlywheelVelocity(ShooterSetpoint shooterSetpoint) {
        setFlywheelVelocity(shooterSetpoint.angularVelocity);
    }

    public void restFlywheel() {
        setFlywheelVelocity(ShooterSetpoint.RESTING);
    }

    public void revFlywheel() {
        setFlywheelVelocity(ShooterSetpoint.REV);
    }

    /** stop all shooter motors */
    public void stopFlywheel() {
        leftBottomFlywheelMotor.stopMotor();
        leftTopFlywheelMotor.stopMotor();
        rightTopFlywheelMotor.stopMotor();
        rightBottomFlywheelMotor.stopMotor();
    }

    /** @return the estimated initial speed of the ball after being shot from the shooter in m/s*/
    public double getFuelExitVelocity() {
        double motorOmega = getFlywheelMotorVelocity().in(RadiansPerSecond);

        double shooterOmega = motorOmega * ShooterConstants.ROTOR_TO_WHEEL_RATIO;

        double wheelTangentialSpeed = shooterOmega * ShooterConstants.WHEEL_RADIUS.in(Meters);
        double rollerTangentialSpeed = shooterOmega * ShooterConstants.ROLLER_RADIUS.in(Meters);

        return (ShooterConstants.FLYWHEEL_EFFICIENCY * (wheelTangentialSpeed + rollerTangentialSpeed)) / 2.0;
    }

    // /** @return the estimated initial speed of the ball after being shot from the shooter in m/s*/
    // public double getFuelSpeedWithCustomEfficiency(double efficiency){
    //     double motorOmega = getShooterMotorVelocity().in(RadiansPerSecond);

    //     double shooterOmega = motorOmega * ShooterConstants.ratio;

    //     double wheelTangentialSpeed = shooterOmega * ShooterConstants.kShooterWheelRadius.in(Meters);
    //     double rollerTangentialSpeed = shooterOmega * ShooterConstants.kShooterRollerRadius.in(Meters);

    //     return efficiency * (wheelTangentialSpeed + rollerTangentialSpeed)/2;
    // }

    /** @return the average angular velocity of the shooter motors measured from all 4 shooter motors*/
    public AngularVelocity getFlywheelMotorVelocity() {
        return RadiansPerSecond.of(
            (leftBottomFlywheelMotor.getVelocity().getValue().in(RadiansPerSecond) +
                leftTopFlywheelMotor.getVelocity().getValue().in(RadiansPerSecond) +
                rightTopFlywheelMotor.getVelocity().getValue().in(RadiansPerSecond) +
                rightBottomFlywheelMotor.getVelocity().getValue().in(RadiansPerSecond)) /
                4
        );
    }

    /** @return true if the shooter motors are at the target velocity (within tolerance), false otherwise*/
    public boolean isAtFlywheelVelocity() {
        return velocityDebouncer.calculate(
            leftBottomFlywheelMotor
                .getClosedLoopReference()
                .isNear(
                    getFlywheelMotorVelocity().in(RotationsPerSecond),
                    ShooterConstants.VELOCITY_TOLERANCE.in(RotationsPerSecond)
                )
        );
    }

    /** @ return true if the hood is at the right pitch within tolerance, false otherwise */
    public boolean isAtHoodPitch() {
        return hoodMotor
            .getClosedLoopReference()
            .isNear(hoodMotor.getPosition().getValueAsDouble(), Units.degreesToRotations(2));
    }

    /** @param dutyCycle the percentage (-1-1) of how much power is sent to the feeder motor*/
    public void setFeederDutyCycle(double dutyCycle) {
        feederMotor.set(dutyCycle);
    }

    /** stops the feeder */
    public void stopFeeder() {
        feederMotor.stopMotor();
    }

    /** @param pitch set the desired angle of the hood*/
    public void setHoodPitch(Angle pitch) {
        //set the hood motor control to the motion magic with a desired position that is the desired angle
        hoodMotor.setControl(hoodMotionMagicVoltage.withPosition(pitch));
    }

    /** @param pitch the desired launch angle of the fuel
     * adjusts the hood such to achieve the desired fuel launch angle
     */
    public void setHoodPitchComplement(Angle pitch) {
        //adjusthood is in terms of shooter angle where the angle of the shooter COM with respect to the horizontal is 0, to get this from trajectory angle must get the complement of the trajectory angle
        //subtract the hood offset which is the angle between the hood COM and the final hood roller
        setHoodPitch(Degrees.of(90).minus(pitch).minus(HoodConstants.PITCH_OFFSET));
    }

    public void setHoodPitchComplement(ShooterSetpoint shooterSetpoint) {
        if (shooterSetpoint.pitch.isEmpty()) {
            throw new IllegalArgumentException("Shooter setpoint pitch cannot be empty");
        }
        setHoodPitchComplement(shooterSetpoint.pitch.get());
    }

    /** sets the desired angle of the hood to the resting angle(fits under the trench) */
    public void restHood() {
        setHoodPitch(HoodConstants.RESTING_PITCH);
    }

    /** @param adjustment how much to adjust by in degrees */
    public void adjustHoodPitch(double adjustment) {
        //make sure the hood setpoint stays within its bounded range
        double targetAngle = Math.max(5, Math.min(manualAngle + adjustment, 45));
        //store the setpoint in the manualAngle member variable
        manualAngle = targetAngle;
        //adjust the hood to the manual setpoint
        setHoodPitch(Degrees.of(targetAngle));
    }

    /** A command to run the shooter motors at a given velocity */
    public class ChangeVelocity extends Command {

        private AngularVelocity velocity;

        /** @param velocity the deisred angular velocity of the shooter
         * this command requires the shooter subsystem (meaning that on initialize it terminates any other command running that also requires the shooter command, and will get terminated if another command that requires shooter is initialized)
         */
        public ChangeVelocity(AngularVelocity velocity) {
            this.velocity = velocity;
            addRequirements(Shooter.this);
        }

        @Override
        public void initialize() {
            //set the shooter target velocity to the desired angular velocity
            //"Shooter.this" is not needed to use setVelocity, it can be accessed directly due to the command being a nested class of the Shooter subsystem
            setFlywheelVelocity(velocity);
        }

        @Override
        public void end(boolean interrupted) {
            //stop the shooter when the command ends
            stopFlywheel();
        }
    }

    /** ChangeState just changes the feeder state (only feeder uses enum states because the rest of the shooter has to dynamically change with auto aim rather than have discrete setpoints)
     * the name is still misleading and should be changed to ChangeFeederState to be more clear
     */
    public class ChangeFeederState extends Command {

        private FeederState feederState;

        /** @param feederState the desired feeder state */
        public ChangeFeederState(FeederState feederState) {
            this.feederState = feederState;
        }

        @Override
        public void initialize() {
            //set the feeder dutyCycle to the percentage in the state object
            Shooter.this.setFeederDutyCycle(feederState.dutyCycle);
        }

        @Override
        public void end(boolean interrupted) {
            //stop the feeder when the command ends
            stopFeeder();
        }
    }
}
