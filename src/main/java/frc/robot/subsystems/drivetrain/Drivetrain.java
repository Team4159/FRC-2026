package frc.robot.subsystems.drivetrain;

import static frc.robot.operator.OperatorConstants.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.AllianceUtil;
import frc.robot.commands.AutoShoot;
import frc.robot.operator.OperatorConstants.DriveFlag;
import frc.robot.operator.OperatorModality;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class Drivetrain extends CommandSwerveDrivetrain {

    public final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleDrive =
        new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(POINT_kP, POINT_kI, POINT_kD)
            .withTargetRateFeedforward(POINT_FEED_FORWARD);
    public final SwerveRequest.FieldCentricFacingAngle trajectoryFacingAngleDrive =
        new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(POINT_kP, POINT_kI, POINT_kD)
            .withTargetRateFeedforward(POINT_FEED_FORWARD);
    public final SwerveRequest.SwerveDriveBrake brakeDrive = new SwerveRequest.SwerveDriveBrake();
    public final SwerveRequest.PointWheelsAt pointDrive = new SwerveRequest.PointWheelsAt();
    public final SwerveRequest.Idle idleDrive = new SwerveRequest.Idle();

    private final Supplier<Double> inputDriveX;
    private final Supplier<Double> inputDriveY;
    private final Supplier<Double> inputRotation;

    private class DriveFlagValue {

        public final boolean defaultValue;
        public boolean value;

        public DriveFlagValue(boolean defaultValue) {
            this.defaultValue = defaultValue;
            this.value = defaultValue;
        }

        public void reset() {
            value = defaultValue;
        }
    }

    private final Map<DriveFlag, DriveFlagValue> driveFlags = new HashMap<DriveFlag, DriveFlagValue>();

    {
        driveFlags.put(DriveFlag.SLOW_MODE, new DriveFlagValue(false));
        driveFlags.put(DriveFlag.DRIVE_ASSIST, new DriveFlagValue(true));
        driveFlags.put(DriveFlag.AUTO_BRAKE, new DriveFlagValue(true));
        driveFlags.put(DriveFlag.INTAKE_ASSIST, new DriveFlagValue(false));
        driveFlags.put(DriveFlag.MANUAL_ALIGN, new DriveFlagValue(false));
    }

    private boolean autoPathAutoShootMode = false;
    private AutoShoot autoShootCommand;

    public Drivetrain(OperatorModality operatorModality) {
        super(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        );
        this.inputDriveX = () -> operatorModality.driveX();
        this.inputDriveY = () -> operatorModality.driveY();
        this.inputRotation = () -> operatorModality.rotation();
    }

    public class DriveFlagToggler extends Command {

        private final DriveFlag driveFlag;

        public DriveFlagToggler(DriveFlag driveFlag) {
            this.driveFlag = driveFlag;
        }

        @Override
        public void initialize() {
            setDriveFlagValue(driveFlag, !getDriveFlagDefaultValue(driveFlag));
        }

        @Override
        public void end(boolean interrupted) {
            setDriveFlagValue(driveFlag, getDriveFlagDefaultValue(driveFlag));
        }
    }

    public Command getDriveCommand(DriveMode driveMode) {
        return new Drive(this, driveMode);
    }

    public void setDriveFlagValue(DriveFlag driveFlag, boolean newValue) {
        driveFlags.get(driveFlag).value = newValue;
    }

    public boolean getDriveFlagValue(DriveFlag driveFlag) {
        return driveFlags.get(driveFlag).value;
    }

    public boolean getDriveFlagDefaultValue(DriveFlag driveFlag) {
        return driveFlags.get(driveFlag).defaultValue;
    }

    public void resetDriveFlags() {
        driveFlags.forEach((key, value) -> value.reset());
    }

    public double getMaxTranslationSpeed() {
        return MAX_TRANSLATION_SPEED * (getDriveFlagValue(DriveFlag.SLOW_MODE) ? SLOW_MODE_TRANSLATION_FACTOR : 1);
    }

    public double getMaxRotationSpeed() {
        return MAX_ROTATION_SPEED * (getDriveFlagValue(DriveFlag.SLOW_MODE) ? SLOW_MODE_ROTATION_FACTOR : 1);
    }

    public Translation2d getInputSpeedTranslation(boolean fieldRelative) {
        return getInputTranslation(fieldRelative).times(getMaxTranslationSpeed());
    }

    public double getInputVelocityX(boolean fieldRelative) {
        return getInputX(fieldRelative) * getMaxTranslationSpeed();
    }

    public double getInputVelocityY(boolean fieldRelative) {
        return getInputY(fieldRelative) * getMaxTranslationSpeed();
    }

    public double getInputSpeedRotation() {
        return getInputRotation() * getMaxRotationSpeed();
    }

    /**
     * @return the field relative translation input (-left joystick y input,
     *         -left
     *         joystick x input), from magnitude range -1 to 1. no deadzone is
     *         applied
     */
    public Translation2d getRawInputTranslation(boolean fieldRelative) {
        Translation2d rawInput = new Translation2d(inputDriveX.get(), inputDriveY.get());
        if (fieldRelative && isInverted()) {
            rawInput = rawInput.times(-1);
        }
        return rawInput;
    }

    /**
     * @return the field relative translation input (-left joystick y input,
     *         -left
     *         joystick x input), from magnitude range -1 to 1. a deadzone is
     *         applied.
     */
    public Translation2d getInputTranslation(boolean fieldRelative) {
        Translation2d rawInput = getRawInputTranslation(fieldRelative);
        Vector<N2> filteredInputVector = rawInput.toVector();
        filteredInputVector = MathUtil.applyDeadband(filteredInputVector, PRIMARY_TRANSLATION_DEADBAND, 1);

        // apply max radius
        filteredInputVector = filteredInputVector.div(PRIMARY_TRANSLATION_RADIUS);

        // apply exponent
        if (filteredInputVector.norm() > 0.0) {
            filteredInputVector = filteredInputVector
                .unit()
                .times(Math.pow(filteredInputVector.norm(), PRIMARY_TRANSLATION_EXPONENT));
        }

        // clamp values
        if (filteredInputVector.norm() > 1) {
            filteredInputVector = filteredInputVector.div(filteredInputVector.norm());
        }

        return new Translation2d(filteredInputVector);
    }

    /**
     * @return the field relative x input (-left joystick y input), from range -1
     *         to
     *         1. a deadzone and quadratic are applied for better control.
     */
    public double getInputX(boolean fieldRelative) {
        return getInputTranslation(fieldRelative).getX();
    }

    /**
     * @return the field relative y input (-left joystick x input), from range -1
     *         to
     *         1. a deadzone and quadratic are applied for better control.
     */
    public double getInputY(boolean fieldRelative) {
        return getInputTranslation(fieldRelative).getY();
    }

    /**
     * @return the field relative rotation input (-right joystick x), from range -1
     *         to 1. no deadzone is applied
     */
    public double getRawInputRotationVelocity() {
        return inputRotation.get();
    }

    /**
     * @return the field relative rotation input (-right joystick x), from range -1
     *         to 1. a deadzone and quadratic are applied for better control.
     */
    public double getInputRotation() {
        double rawInput = getRawInputRotationVelocity();
        double filteredInput = MathUtil.applyDeadband(Math.abs(rawInput), PRIMARY_ROTATION_DEADBAND, 1);
        return Math.abs(Math.pow(filteredInput, PRIMARY_ROTATION_EXPONENT)) * Math.signum(rawInput);
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(() -> getState().Pose, this::resetPose, this::followPath, true, this, trajLogger);
    }

    /** set the AutoAim command to be used for shooting while moving during auto */
    public void setAutonomousAutoShootCommand(AutoShoot autoShootCommand) {
        this.autoShootCommand = autoShootCommand;
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * if autoPathAutoAimMode is true and the setAutonomousAutoAimCommand() method
     * was used to set the autoAimCommand, it will use the omega from the auto aim
     * command to aim at the hub
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        // current robot pose
        var pose = getState().Pose;
        // choreo calculated target speeds (time based)
        var targetSpeeds = sample.getChassisSpeeds();
        // calculate translation velocities with target speeds and PID to correct for
        // error
        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(pose.getX(), sample.x);
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(pose.getY(), sample.y);

        // omega is calculated differently depending on the mode
        // if autoPathAutoAimMode is true and the setAutonomousAutoAimCommand() method
        // was used to set the autoAimCommand, it will use the omega from the auto aim
        // command to aim at the hub
        // otherwise it will use target speeds and PID
        if (autoPathAutoShootMode && autoShootCommand != null) {
            // get the desired omega directly from the auto aim controller(instead of
            // calculated speeds and PID)
            targetSpeeds.omegaRadiansPerSecond = autoShootCommand.getDesiredOmega();
        } else {
            m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
            // get desired omega from calculated speeds and PID
            targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
                pose.getRotation().getRadians(),
                sample.heading
            );
        }
        // send the calculated speeds to the drivetrain
        setControl(
            m_pathApplyFieldSpeeds
                .withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    /**
     * @param autoPathAutoShootMode if true the robot will run autoaim along the auto
     *                            trajectory
     *                            a value of true will activate the AutoAim command
     *                            and a value of false will cancel it. it will also
     *                            schedule and cancel the auto aim command object
     *                            stored in the Drivetrain class.
     *                            I hate this implementation but I have negative
     *                            intelligence
     */
    public void setAutoPathAutoShootMode(boolean autoPathAutoShootMode) {
        this.autoPathAutoShootMode = autoPathAutoShootMode;
        if (autoPathAutoShootMode) {
            CommandScheduler.getInstance().schedule(autoShootCommand);
        } else {
            CommandScheduler.getInstance().cancel(autoShootCommand);
        }
    }

    /**
     * @return {@Code true} if there is no joystick input and the desired rotation
     *         has been reached
     */
    public boolean isDriveIdle() {
        return getInputTranslation(false).getNorm() == 0.0 && getInputRotation() == 0.0;
    }

    public boolean canAutoBrake() {
        return getDriveFlagValue(DriveFlag.AUTO_BRAKE) && isDriveIdle();
    }

    private boolean isInverted() {
        return AllianceUtil.getAlliance() == Alliance.Red;
    }
}
