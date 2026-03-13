package frc.robot.subsystems;

import static frc.robot.Constants.OperatorConstants.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.FieldUtil;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.DriveMode;
import frc.robot.commands.AutoAim;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class Drivetrain extends CommandSwerveDrivetrain {

    public final Pigeon2 pigeon = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);

    public final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(kAimKP, kAimKI, kAimKD)
            .withTargetRateFeedforward(kAimFeedForward);
    public final SwerveRequest.RobotCentricFacingAngle robotCentricFacingAngleDrive = new SwerveRequest.RobotCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(kAimKP, kAimKI, kAimKD)
            .withTargetRateFeedforward(kAimFeedForward);
    public final SwerveRequest.FieldCentricFacingAngle trajectoryFacingAngleDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(kAimKP, kAimKI, kAimKD)
            .withTargetRateFeedforward(kAimFeedForward);
    public final SwerveRequest.SwerveDriveBrake brakeDrive = new SwerveRequest.SwerveDriveBrake();
    public final SwerveRequest.PointWheelsAt pointDrive = new SwerveRequest.PointWheelsAt();
    public final SwerveRequest.Idle idleDrive = new SwerveRequest.Idle();

    public final Trigger crashTrigger = new Trigger(
            () -> Math.hypot(pigeon.getAccelerationX().getValue().in(MetersPerSecondPerSecond),
                    pigeon.getAccelerationY().getValue().in(MetersPerSecondPerSecond)) >= FieldConstants.g * 3.0);

    private final Supplier<Double> inputX;
    private final Supplier<Double> inputY;
    private final Supplier<Double> inputRotationVelocity;
    private final Supplier<Double> inputRotationX;
    private final Supplier<Double> inputRotationY;

    private boolean reduceSpeedEnabled = false;
    private boolean driveAssistEnabled = true;

    private boolean autoPathAutoAimMode = false;
    private AutoAim autoAimCommand;

    private boolean autoBrakeEnabled = true;

    private Optional<Rotation2d> desiredRotation = Optional.empty();

    public Drivetrain(CommandXboxController controller) {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight,
                TunerConstants.BackLeft, TunerConstants.BackRight);
        this.inputX = () -> -controller.getLeftY();
        this.inputY = () -> -controller.getLeftX();
        this.inputRotationVelocity = () -> -controller.getRightX();
        this.inputRotationX = () -> -controller.getRightY();
        this.inputRotationY = () -> -controller.getRightX();
    }

    public class Drive extends Command {
        private final BooleanSupplier robotRelativeSupplier;
        private final Supplier<SwerveRequest> driveSupplier;

        public Drive(DriveMode driveMode, BooleanSupplier robotRelativeSupplier) {
            this.robotRelativeSupplier = robotRelativeSupplier;
            this.driveSupplier = getDriveSupplier(driveMode);
            addRequirements(Drivetrain.this);
        }

        public Drive(DriveMode driveMode) {
            this(driveMode, () -> false);
        }

        @Override
        public void execute() {
            setControl(driveSupplier.get());
        }

        private SwerveRequest getIntakeDrive(Rotation2d rotationOffset) {
            if (getInputTranslation(true).getNorm() < kPrimaryIntakeRotationInputDeadzone) {
                return fieldCentricDrive
                        .withVelocityX(getInputSpeedX(true))
                        .withVelocityY(getInputSpeedY(true));
            }
            Rotation2d rotation = new Rotation2d(getInputX(true), getInputY(true))
                    .plus(rotationOffset);
            return fieldCentricFacingAngleDrive
                    .withVelocityX(getInputSpeedX(true))
                    .withVelocityY(getInputSpeedY(true))
                    .withTargetDirection(rotation);
        }

        private Optional<ChassisSpeeds> driveAssist() {
            if (!driveAssistEnabled || !DriverStation.isTeleop()) {
                return Optional.empty();
            }
            Pose2d robotPose = getState().Pose;

            // trench assist
            var trenchZone = FieldUtil.getPoseTrenchZone(robotPose);
            if (trenchZone.isPresent()) {
                Distance errorX = trenchZone.get().x.minus(robotPose.getMeasureX());
                Distance errorY = trenchZone.get().y.minus(robotPose.getMeasureY());

                boolean hasPassed = !errorX.isNear(Meters.zero(), OperatorConstants.kTrenchAssistPassPositionTolerance);
                boolean isNotApproaching = (Math.signum(getInputX(true)) == -Math.signum(errorX.magnitude()))
                        || Math.abs(getInputX(true)) <= OperatorConstants.kTrenchAssistApproachInputTolerance;
                if (hasPassed && isNotApproaching) {
                    return Optional.empty();
                }

                double vy = kTrenchAssistAlignStrength * getMaxTranslationSpeed() * Math.signum(errorY.magnitude())
                        * Math.abs(getInputX(true));
                double influence = OperatorConstants.kTrenchAssistAlignInfluence * getInputSpeedY(true);
                boolean insideTolerance = errorY.isNear(Meters.zero(),
                        OperatorConstants.kTrenchAssistAlignPositionTolerance);
                boolean againstAlignment = (influence >= Math.abs(vy));
                if (insideTolerance || againstAlignment) {
                    vy = 0.0;
                }
                vy += influence;

                return Optional.of(new ChassisSpeeds(
                        0.0,
                        vy,
                        0.0));
            }

            return Optional.empty();
        }

        private Supplier<SwerveRequest> getDriveSupplier(DriveMode driveMode) {
            return () -> switch (driveMode) {
                case FREE -> {
                    if (canAutoBrake()) {
                        yield brakeDrive;
                    }
                    if (robotRelativeSupplier.getAsBoolean()) {
                        if (desiredRotation.isPresent()) {
                            yield robotCentricFacingAngleDrive.withVelocityX(getInputSpeedX(true))
                                    .withVelocityY(getInputSpeedY(false))
                                    .withTargetDirection(desiredRotation.get());
                        }
                        yield robotCentricDrive.withVelocityX(getInputSpeedX(false))
                                .withVelocityY(getInputSpeedY(false))
                                .withRotationalRate(getInputSpeedRotation());
                    }
                    var assistSpeed = driveAssist();
                    var velocityY = assistSpeed.isEmpty() ? getInputSpeedY(true)
                            : assistSpeed.get().vyMetersPerSecond;
                    if (desiredRotation.isPresent()) {
                        yield fieldCentricFacingAngleDrive.withVelocityX(getInputSpeedX(true))
                                .withVelocityY(velocityY)
                                .withTargetDirection(desiredRotation.get());
                    }
                    yield fieldCentricDrive.withVelocityX(getInputSpeedX(true))
                            .withVelocityY(velocityY)
                            .withRotationalRate(getInputSpeedRotation());
                }
                case BRAKE -> brakeDrive;
                case POINT -> pointDrive
                        .withModuleDirection(new Rotation2d(getInputX(false), getInputY(false)));
                case IDLE -> idleDrive;
                case MANUAL_ALIGN -> {

                    // 8 direction behavior
                    // Translation2d rawInput =
                    // getRawInputTranslation(!robotRelativeSupplier.getAsBoolean());
                    // double x = Math.signum(MathUtil.applyDeadband(rawInput.getX(),
                    // kPrimaryAlignModeDeadband, 1));
                    // double y = Math.signum(MathUtil.applyDeadband(rawInput.getY(),
                    // kPrimaryAlignModeDeadband, 1));
                    // if (x != 0 && y != 0) {
                    // x /= Math.sqrt(2);
                    // y /= Math.sqrt(2);
                    // y = 0;
                    // }

                    // 4 direction/axis behavior
                    Translation2d input = getInputTranslation(true);
                    double x = 0;
                    double y = 0;
                    if (input.getNorm() >= 0.0) {
                        if (Math.abs(input.getAngle().getCos()) >= input.getNorm() / Math.sqrt(2)) {
                            x = Math.signum(input.getX());
                        } else {
                            y = Math.signum(input.getY());
                        }
                    }

                    double velocityX = x * getMaxTranslationSpeed();
                    double velocityY = y * getMaxTranslationSpeed();
                    double rotationalRate = getInputRotationVelocity() * kMaxRotationSpeed;
                    if (reduceSpeedEnabled) {
                        velocityX *= kPrimaryAlignModeSpeedTranslationFactor;
                        velocityY *= kPrimaryAlignModeSpeedTranslationFactor;
                        rotationalRate *= kPrimaryAlignModeSpeedRotationFactor;
                    }

                    if (robotRelativeSupplier.getAsBoolean()) {
                        yield robotCentricDrive.withVelocityX(velocityX)
                                .withVelocityY(velocityY)
                                .withRotationalRate(rotationalRate);
                    }
                    yield fieldCentricDrive.withVelocityX(velocityX)
                            .withVelocityY(velocityY)
                            .withRotationalRate(rotationalRate);
                }
                case INTAKE_FORWARD -> {
                    yield getIntakeDrive(Rotation2d.kZero);
                }
                case INTAKE_LEFT -> {
                    yield getIntakeDrive(new Rotation2d(Degrees.of(45.0)));
                }
                case INTAKE_RIGHT -> {
                    yield getIntakeDrive(new Rotation2d(Degrees.of(-45.0)));
                }
                case RADIAL -> {
                    if (canAutoBrake()) {
                        yield brakeDrive;
                    }
                    double radialInput = MathUtil.applyDeadband(inputX.get(), kPrimaryRadialModeDeadband, 1);
                    double tangentialInput = MathUtil.applyDeadband(inputY.get(), kPrimaryRadialModeDeadband, 1);
                    Pose2d robotPose = getState().Pose;
                    Pose2d hubPose = FieldConstants.hubLocations.get(DriverStation.getAlliance().orElse(Alliance.Blue));
                    Translation2d radialVector = new Translation2d(
                            hubPose.getX() - robotPose.getX(),
                            hubPose.getY() - robotPose.getY());
                    if (radialVector.getNorm() <= 1e-3) {
                        radialVector = new Translation2d(0.0, 0.0);
                    } else {
                        radialVector = radialVector.div(radialVector.getNorm());
                    }
                    Translation2d tangentialVector = new Translation2d(
                            radialVector.getY(),
                            radialVector.getX());
                    double velocityX = getMaxTranslationSpeed()
                            * (radialInput * radialVector.getX() - tangentialInput * tangentialVector.getX());
                    double velocityY = getMaxTranslationSpeed()
                            * (radialInput * radialVector.getY() + tangentialInput * tangentialVector.getY());
                    double velocityMagnitude = Math.hypot(velocityX, velocityY);
                    if (velocityMagnitude > getMaxTranslationSpeed()) {
                        double correctingFactor = getMaxTranslationSpeed() / velocityMagnitude;
                        velocityX *= correctingFactor;
                        velocityY *= correctingFactor;
                    }
                    if (desiredRotation.isPresent()) {
                        yield fieldCentricFacingAngleDrive
                                .withVelocityX(velocityX)
                                .withVelocityY(velocityY)
                                .withTargetDirection(desiredRotation.get());
                    }
                    yield fieldCentricDrive
                            .withVelocityX(velocityX)
                            .withVelocityY(velocityY);
                }
            };
        }
    }

    public void enableReduceSpeed(boolean enabled) {
        this.reduceSpeedEnabled = enabled;
    }

    public void enableDriveAssist(boolean enabled) {
        this.driveAssistEnabled = enabled;
    }

    public void enableAutoBrake(boolean enabled) {
        this.autoBrakeEnabled = enabled;
    }

    public void setDesiredRotation(Rotation2d desiredRotation) {
        this.desiredRotation = Optional.of(desiredRotation);
    }

    public void clearDesiredRotation() {
        this.desiredRotation = Optional.empty();
    }

    public double getMaxTranslationSpeed() {
        return kMaxTranslationSpeed * (reduceSpeedEnabled ? kPrimaryReduceSpeedTranslationFactor : 1);
    }

    public double getMaxRotationSpeed() {
        return kMaxRotationSpeed * (reduceSpeedEnabled ? kPrimaryReduceSpeedRotationFactor : 1);
    }

    public double getInputSpeedX(boolean fieldRelative) {
        return getInputX(fieldRelative) * getMaxTranslationSpeed();
    }

    public double getInputSpeedY(boolean fieldRelative) {
        return getInputY(fieldRelative) * getMaxTranslationSpeed();
    }

    public double getInputSpeedRotation() {
        return getInputRotationVelocity() * getMaxRotationSpeed();
    }

    /**
     * @return the field relative translation input (-left joystick y input,
     *         -left
     *         joystick x input), from magnitude range -1 to 1. no deadzone is
     *         applied
     */
    public Translation2d getRawInputTranslation(boolean fieldRelative) {
        Translation2d rawInput = new Translation2d(inputX.get(), inputY.get());
        if (fieldRelative && DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
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
        filteredInputVector = MathUtil.applyDeadband(filteredInputVector, kPrimaryTranslationDeadband, 1);

        // apply max radius
        filteredInputVector = filteredInputVector.div(kPrimaryTranslationRadius);

        // apply exponent
        if (filteredInputVector.norm() > 0.0) {
            filteredInputVector = filteredInputVector.unit()
                    .times(Math.pow(filteredInputVector.norm(), kPrimaryTranslationExponent));
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
        return inputRotationVelocity.get();
    }

    /**
     * @return the field relative rotation input (-right joystick x), from range -1
     *         to 1. a deadzone and quadratic are applied for better control.
     */
    public double getInputRotationVelocity() {
        double rawInput = getRawInputRotationVelocity();
        double filteredInput = MathUtil.applyDeadband(Math.abs(rawInput),
                kPrimaryRotationDeadband, 1);
        return Math.abs(Math.pow(filteredInput, kPrimaryRotationExponent)) * Math.signum(rawInput);
    }

    public Rotation2d getRawInputRotation() {
        return new Rotation2d(inputRotationX.get(), inputRotationY.get());
    }

    public Optional<Rotation2d> getInputRotation() {
        if (Math.hypot(inputRotationX.get(), inputRotationY.get()) < kPrimaryRotationDeadband) {
            return Optional.empty();
        }
        Rotation2d filteredInput = getRawInputRotation();
        if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
            filteredInput = filteredInput.plus(Rotation2d.k180deg);
        }
        return Optional.of(filteredInput);
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {
        });
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
                () -> getState().Pose,
                this::resetPose,
                this::followPath,
                true,
                this,
                trajLogger);
    }

    /** set the AutoAim command to be used for shooting while moving during auto */
    public void setAutonomousAutoAimCommand(AutoAim autoAimCommand) {
        this.autoAimCommand = autoAimCommand;
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
        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(
                pose.getX(), sample.x);
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(
                pose.getY(), sample.y);

        // omega is calculated differently depending on the mode
        // if autoPathAutoAimMode is true and the setAutonomousAutoAimCommand() method
        // was used to set the autoAimCommand, it will use the omega from the auto aim
        // command to aim at the hub
        // otherwise it will use target speeds and PID
        if (autoPathAutoAimMode && autoAimCommand != null) {
            // get the desired omega directly from the auto aim controller(instead of
            // calculated speeds and PID)
            targetSpeeds.omegaRadiansPerSecond = autoAimCommand.getDesiredOmega();
        } else {
            m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
            // get desired omega from calculated speeds and PID
            targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
                    pose.getRotation().getRadians(), sample.heading);
        }
        // send the calculated speeds to the drivetrain
        setControl(
                m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
                        .withWheelForceFeedforwardsX(sample.moduleForcesX())
                        .withWheelForceFeedforwardsY(sample.moduleForcesY()));
    }

    /**
     * @param autoPathAutoAimMode if true the robot will run autoaim along the auto
     *                            trajectory
     *                            a value of true will activate the AutoAim command
     *                            and a value of false will cancel it. it will also
     *                            schedule and cancel the auto aim command object
     *                            stored in the Drivetrain class.
     *                            I hate this implementation but I have negative
     *                            intelligence
     */
    public void setAutoPathAutoAimMode(boolean autoPathAutoAimMode) {
        this.autoPathAutoAimMode = autoPathAutoAimMode;
        if (autoPathAutoAimMode) {
            CommandScheduler.getInstance().schedule(autoAimCommand);
        } else {
            CommandScheduler.getInstance().cancel(autoAimCommand);
        }
    }

    /**
     * @return {@Code true} if there is no joystick input and the desired rotation
     *         has been reached
     */
    public boolean isDriveIdle() {
        boolean noInputRotation = true;
        if (desiredRotation.isPresent()) {
            noInputRotation = getInputRotation().isEmpty();
        }
        return getInputTranslation(false).getNorm() == 0.0 && getInputRotationVelocity() == 0.0 && noInputRotation;
    }

    public boolean isAtDesiredRotation() {
        if (desiredRotation.isEmpty()) {
            return false;
        }
        return Math.abs(getState().Pose.getRotation().minus(desiredRotation.get()).getMeasure()
                .baseUnitMagnitude()) < kPrimaryAutoBrakeReachedDesiredAngleTolerance
                        .baseUnitMagnitude();
    }

    public boolean canAutoBrake() {
        boolean atDesiredRotation = (desiredRotation.isEmpty() ? true : isAtDesiredRotation());
        return autoBrakeEnabled && isDriveIdle() && atDesiredRotation;
    }

}
