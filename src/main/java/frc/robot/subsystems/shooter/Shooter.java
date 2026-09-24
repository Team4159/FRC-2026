package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.FeederConstants.FeederState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterSetpoint;

public class Shooter extends SubsystemBase {

    //Phoenix control requests
    //this allows for control systems to be run on the motor controllers (less work for roborio) because CTRE is actually good at this (unlike REV, for REV motors just use a WPILIB PID)

    //Motion Magic® is a CTRE control request that is a profiled PID controller that takes a max velocity, acceleration, and jerk into account for smoother motion control, works very well for pivots
    private final MotionMagicVoltage hoodMotionMagicVoltage;
    //velocity voltage is just a regular voltage based PID for a velocity setpoint
    private final VelocityVoltage flywheelVelocityVoltage;

    //the current manual angle setpoint in degrees
    private double manualAngle = 5.0;

    public Shooter() {
        //initialize the control requests(the setpoints are changed later and are currently meaningless)
        flywheelVelocityVoltage = new VelocityVoltage(0.0);
        hoodMotionMagicVoltage = new MotionMagicVoltage(0.0);

        //set the hood to the resting position
        //when making commands for the shooter the hood should always be set back to resting position when done so the robot can go under the trench
        restHood();
        stopFlywheel();
    }

    @Override
    public void periodic() {
        //just a bunch of smartdashboard logging used for tuning
        SmartDashboard.putNumber(
            "hood position",
            Units.rotationsToDegrees(HoodConstants.MOTOR.getPosition().getValueAsDouble())
        );
        SmartDashboard.putNumber("hood target position", hoodMotionMagicVoltage.Position);
        SmartDashboard.putNumber("manual hood target", manualAngle);
        SmartDashboard.putNumber("shooter velocity", getFlywheelMotorVelocityTarget().in(RPM));
        SmartDashboard.putNumber("shooter velocity target", flywheelVelocityVoltage.getVelocityMeasure().in(RPM));
    }

    /** @param deisredAngularVelocity the desired angular velocity of the motors */
    public void setFlywheelVelocity(AngularVelocity desiredAngularVelocity) {
        //set the velocity target of the velocity voltage to the desired angular velocity
        flywheelVelocityVoltage.withVelocity(desiredAngularVelocity.in(RotationsPerSecond));
        //set the control of the motors to the velocityVoltage
        for (TalonFX flywheelMotor : FlywheelConstants.MOTORS) {
            flywheelMotor.setControl(flywheelVelocityVoltage);
        }
    }

    public void setFlywheelMotorVelocity(ShooterSetpoint shooterSetpoint) {
        setFlywheelVelocity(shooterSetpoint.angularVelocity);
    }

    public void restFlywheel() {
        setFlywheelMotorVelocity(ShooterSetpoint.RESTING);
    }

    public void revFlywheel() {
        setFlywheelMotorVelocity(ShooterSetpoint.REV);
    }

    /** stop all shooter motors */
    public void stopFlywheel() {
        for (TalonFX flywheelMotor : FlywheelConstants.MOTORS) {
            flywheelMotor.stopMotor();
        }
    }

    /** @return the estimated initial speed of the ball after being shot from the shooter in m/s*/
    public double getFuelExitVelocity() {
        double motorOmega = getFlywheelMotorVelocityTarget().in(RadiansPerSecond);

        double shooterOmega = motorOmega * FlywheelConstants.ROTOR_TO_WHEEL_RATIO;

        double wheelTangentialSpeed = shooterOmega * FlywheelConstants.WHEEL_RADIUS.in(Meters);
        double rollerTangentialSpeed = shooterOmega * FlywheelConstants.ROLLER_RADIUS.in(Meters);

        return (FlywheelConstants.SHOOT_EFFICIENCY * (wheelTangentialSpeed + rollerTangentialSpeed)) / 2.0;
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
    public AngularVelocity getFlywheelMotorVelocityTarget() {
        double sum = 0.0;
        for (TalonFX flywheelMotor : FlywheelConstants.MOTORS) {
            sum += flywheelMotor.getVelocity().getValue().in(RadiansPerSecond);
        }
        return RadiansPerSecond.of(sum / FlywheelConstants.MOTORS.length);
    }

    /** @return true if the shooter motors are at the target velocity (within tolerance), false otherwise*/
    public boolean isAtFlywheelVelocity() {
        return getFlywheelMotorVelocityTarget().isNear(
            flywheelVelocityVoltage.getVelocityMeasure(),
            FlywheelConstants.VELOCITY_TARGET_TOLERANCE
        );
    }

    /** @ return true if the hood is at the right pitch within tolerance, false otherwise */
    public boolean isAtHoodPitch() {
        return HoodConstants.MOTOR.getPosition()
            .getValue()
            .isNear(hoodMotionMagicVoltage.getPositionMeasure(), Degrees.of(2));
    }

    /** @param dutyCycle the percentage (-1-1) of how much power is sent to the feeder motor*/
    public void setFeederDutyCycle(double dutyCycle) {
        FeederConstants.MOTOR.set(dutyCycle);
    }

    /** stops the feeder */
    public void stopFeeder() {
        FeederConstants.MOTOR.stopMotor();
    }

    /** @param pitch set the desired angle of the hood*/
    public void setHoodPitch(Angle pitch) {
        //set the hood motor control to the motion magic with a desired position that is the desired angle
        HoodConstants.MOTOR.setControl(hoodMotionMagicVoltage.withPosition(pitch));
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
