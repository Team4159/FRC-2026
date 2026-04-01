package frc.robot.subsystems;

import static frc.robot.Constants.OperatorConstants.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.HashMap;
import java.util.Map;
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
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.PoseUtil;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.DriveFlag;
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
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(kPointKP, kPointKI, kPointKD)
            .withTargetRateFeedforward(kPointFeedForward);
    public final SwerveRequest.RobotCentricFacingAngle robotCentricFacingAngleDrive = new SwerveRequest.RobotCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(kPointKP, kPointKI, kPointKD)
            .withTargetRateFeedforward(kPointFeedForward);
    public final SwerveRequest.FieldCentricFacingAngle trajectoryFacingAngleDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(kPointKP, kPointKI, kPointKD)
            .withTargetRateFeedforward(kPointFeedForward);
    public final SwerveRequest.SwerveDriveBrake brakeDrive = new SwerveRequest.SwerveDriveBrake();
    public final SwerveRequest.PointWheelsAt pointDrive = new SwerveRequest.PointWheelsAt();
    public final SwerveRequest.Idle idleDrive = new SwerveRequest.Idle();

    public final Trigger crashTrigger = new Trigger(this::isCrashing);
    public final Trigger slipTrigger = new Trigger(this::isSlipping);
    private final LinearFilter slippingBucketFilter = LinearFilter.movingAverage(25);

    private final Supplier<Double> inputX;
    private final Supplier<Double> inputY;
    private final Supplier<Double> inputRotationVelocity;
    private final Supplier<Double> inputRotationX;
    private final Supplier<Double> inputRotationY;

    private final ChassisSpeeds estimatedRealChassisSpeeds = new ChassisSpeeds(0, 0,
            pigeon.getAngularVelocityYWorld().getValueAsDouble());
    private final LinearFilter estimatedRealChassisSpeedXFilter = LinearFilter.movingAverage(5);
    private final LinearFilter estimatedRealChassisSpeedYFilter = LinearFilter.movingAverage(5);
    private Pose2d lastPose = getState().Pose;

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
    private Optional<Rotation2d> externalDesiredRotation = Optional.empty();

    private boolean autoPathAutoAimMode = false;
    private AutoAim autoAimCommand;

    public Drivetrain(CommandXboxController controller) {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight,
                TunerConstants.BackLeft, TunerConstants.BackRight);
        this.inputX = () -> -controller.getLeftY();
        this.inputY = () -> -controller.getLeftX();
        this.inputRotationVelocity = () -> -controller.getRightX();
        this.inputRotationX = () -> -controller.getRightY();
        this.inputRotationY = () -> -controller.getRightX();
    }

    @Override
    public void periodic() {
        Pose2d currentPose = getState().Pose;
        Translation2d displacement = currentPose.getTranslation().minus(lastPose.getTranslation());
        lastPose = currentPose;
        estimatedRealChassisSpeeds.vxMetersPerSecond = estimatedRealChassisSpeedXFilter
                .calculate(displacement.getX() / 0.02);
        estimatedRealChassisSpeeds.vyMetersPerSecond = estimatedRealChassisSpeedYFilter
                .calculate(displacement.getY() / 0.02);
        estimatedRealChassisSpeeds.omegaRadiansPerSecond = pigeon.getAngularVelocityYWorld().getValueAsDouble();
        isSlipping();
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

        private SwerveRequest getTeleopDrive(boolean robotRelative) {
            if (canAutoBrake()) {
                return brakeDrive;
            }

            double maxTranslationSpeed = getMaxTranslationSpeed();
            double maxRotationSpeed = getMaxRotationSpeed();
            if (getDriveFlagValue(DriveFlag.SLOW_MODE) && getDriveFlagValue(DriveFlag.MANUAL_ALIGN)) {
                maxTranslationSpeed *= kPrimaryAlignModeSpeedTranslationFactor;
                maxRotationSpeed *= kPrimaryAlignModeSpeedRotationFactor;
            }

            Translation2d inputSpeedTranslation;
            double inputSpeedRotation = getInputRotationVelocity() * maxRotationSpeed;
            if (getDriveFlagValue(DriveFlag.MANUAL_ALIGN)) {
                Translation2d input = getInputTranslation(!robotRelative);
                double x = 0;
                double y = 0;
                if (input.getNorm() >= 0.0) {
                    if (Math.abs(input.getAngle().getCos()) >= input.getNorm() / Math.sqrt(2)) {
                        x = Math.signum(input.getX());
                    } else {
                        y = Math.signum(input.getY());
                    }
                }
                inputSpeedTranslation = new Translation2d(
                        x * maxTranslationSpeed,
                        y * maxTranslationSpeed);
            } else {
                inputSpeedTranslation = getInputSpeedTranslation(!robotRelative);
            }

            Optional<Rotation2d> desiredRotation = Optional.empty();
            if (externalDesiredRotation.isPresent()) {
                desiredRotation = Optional.of(externalDesiredRotation.get());
            } else if (getDriveFlagValue(DriveFlag.INTAKE_ASSIST)
                    && getInputTranslation(true).getNorm() >= kPrimaryIntakeRotationInputDeadzone) {
                Angle angle;
                int angleSign = (int) Math.signum(getInputRotationVelocity());
                if (angleSign > 0) {
                    angle = Degrees.of(45.0);
                } else if (angleSign < 0) {
                    angle = Degrees.of(-45.0);
                } else {
                    angle = Degrees.of(0.0);
                }
                desiredRotation = Optional.of(new Rotation2d(inputSpeedTranslation.getX(), inputSpeedTranslation.getY())
                        .plus(new Rotation2d(angle)));
            }

            if (robotRelative) {
                if (desiredRotation.isPresent()) {
                    return robotCentricFacingAngleDrive.withVelocityX(inputSpeedTranslation.getX())
                            .withVelocityY(inputSpeedTranslation.getY())
                            .withTargetDirection(desiredRotation.get());
                }
                return robotCentricDrive.withVelocityX(inputSpeedTranslation.getX())
                        .withVelocityY(inputSpeedTranslation.getY())
                        .withRotationalRate(inputSpeedRotation);
            }

            var assistSpeed = driveAssist();
            var velocityY = assistSpeed.isEmpty() ? inputSpeedTranslation.getY()
                    : assistSpeed.get().vyMetersPerSecond;
            if (desiredRotation.isPresent()) {
                return fieldCentricFacingAngleDrive.withVelocityX(inputSpeedTranslation.getX())
                        .withVelocityY(velocityY)
                        .withTargetDirection(desiredRotation.get());
            }
            return fieldCentricDrive.withVelocityX(inputSpeedTranslation.getX())
                    .withVelocityY(velocityY)
                    .withRotationalRate(inputSpeedRotation);
        }

        private Optional<ChassisSpeeds> driveAssist() {
            if (!getDriveFlagValue(DriveFlag.DRIVE_ASSIST) || !DriverStation.isTeleop()
                    || getDriveFlagValue(DriveFlag.MANUAL_ALIGN)) {
                return Optional.empty();
            }

            Pose2d robotPose = getState().Pose;

            // trench assist
            var trenchZone = PoseUtil.getPoseTrenchZone(robotPose);
            if (trenchZone.isPresent()) {
                Translation2d trenchFocus = trenchZone.get().focus;

                var leftExtentDiagonal = Pair.of(
                        new Translation2d(kChassisSizeX.div(2).plus(HopperConstants.kHopperExtent),
                                kBumperSizeY.div(2)),
                        new Translation2d(kBumperSizeX.div(-2), kBumperSizeY.div(-2)));
                var rotatedLeftExtentDiagonal = Pair.of(
                        leftExtentDiagonal.getFirst().rotateAround(Translation2d.kZero, robotPose.getRotation()),
                        leftExtentDiagonal.getSecond().rotateAround(Translation2d.kZero, robotPose.getRotation()));
                var rotatedRightExtentDiagonal = Pair.of(
                        new Translation2d(leftExtentDiagonal.getFirst().getX(), -leftExtentDiagonal.getFirst().getY())
                                .rotateAround(Translation2d.kZero, robotPose.getRotation()),
                        new Translation2d(leftExtentDiagonal.getSecond().getX(), -leftExtentDiagonal.getSecond().getY())
                                .rotateAround(Translation2d.kZero, robotPose.getRotation()));
                double leftDiagonalVertical = Math.abs(
                        rotatedLeftExtentDiagonal.getFirst().getY() - rotatedLeftExtentDiagonal.getSecond().getY());
                double rightDiagonalVertical = Math.abs(
                        rotatedRightExtentDiagonal.getFirst().getY() - rotatedRightExtentDiagonal.getSecond().getY());
                var mostVerticalDiagonal = (leftDiagonalVertical > rightDiagonalVertical) ? rotatedLeftExtentDiagonal
                        : rotatedRightExtentDiagonal;
                Distance focusOffset = (mostVerticalDiagonal.getFirst().getMeasureY()
                        .plus(mostVerticalDiagonal.getSecond().getMeasureY())).div(-2);
                Translation2d alignFocus = trenchFocus.plus(new Translation2d(Meters.of(0.0),
                        focusOffset));

                Distance errorX = alignFocus.getMeasureX().minus(robotPose.getMeasureX());
                Distance errorY = alignFocus.getMeasureY().minus(robotPose.getMeasureY());
                Distance localErrorY = (trenchFocus.getY() < FieldConstants.kAllianceHeight.baseUnitMagnitude() / 2)
                        ? errorY.copy()
                        : errorY.times(-1);

                boolean hasPassed = !errorX.isNear(Meters.zero(), OperatorConstants.kTrenchAssistPassPositionTolerance);
                boolean isNotApproaching = (Math.signum(getInputX(true)) == -Math.signum(errorX.magnitude()))
                        || Math.abs(getInputX(true)) <= OperatorConstants.kTrenchAssistApproachInputTolerance;
                if (hasPassed && isNotApproaching) {
                    return Optional.empty();
                }

                double vy = kTrenchAssistAlignStrength * Math.signum(errorY.magnitude())
                        * Math.abs(getInputSpeedX(true));
                double influence = OperatorConstants.kTrenchAssistAlignInfluence * getInputSpeedY(true);
                boolean aligned = localErrorY
                        .baseUnitMagnitude() >= -OperatorConstants.kTrenchAssistAlignPositionInnerTolerance
                                .baseUnitMagnitude()
                        && localErrorY.baseUnitMagnitude() <= OperatorConstants.kTrenchAssistAlignPositionOuterTolerance
                                .baseUnitMagnitude();
                boolean againstAlignment = (influence >= Math.abs(vy));
                if (aligned || againstAlignment) {
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
                case TELEOP -> getTeleopDrive(robotRelativeSupplier.getAsBoolean());
                case BRAKE -> brakeDrive;
                case POINT -> pointDrive
                        .withModuleDirection(new Rotation2d(getInputX(false), getInputY(false)));
                case IDLE -> idleDrive;
                // case RADIAL -> {
                // if (canAutoBrake()) {
                // yield brakeDrive;
                // }
                // double radialInput = MathUtil.applyDeadband(inputX.get(),
                // kPrimaryRadialModeDeadband, 1);
                // double tangentialInput = MathUtil.applyDeadband(inputY.get(),
                // kPrimaryRadialModeDeadband, 1);
                // Pose2d robotPose = getState().Pose;
                // Pose2d hubPose =
                // FieldConstants.hubLocations.get(DriverStation.getAlliance().orElse(Alliance.Blue));
                // Translation2d radialVector = new Translation2d(
                // hubPose.getX() - robotPose.getX(),
                // hubPose.getY() - robotPose.getY());
                // if (radialVector.getNorm() <= 1e-3) {
                // radialVector = new Translation2d(0.0, 0.0);
                // } else {
                // radialVector = radialVector.div(radialVector.getNorm());
                // }
                // Translation2d tangentialVector = new Translation2d(
                // radialVector.getY(),
                // radialVector.getX());
                // double velocityX = getMaxTranslationSpeed()
                // * (radialInput * radialVector.getX() - tangentialInput *
                // tangentialVector.getX());
                // double velocityY = getMaxTranslationSpeed()
                // * (radialInput * radialVector.getY() + tangentialInput *
                // tangentialVector.getY());
                // double velocityMagnitude = Math.hypot(velocityX, velocityY);
                // if (velocityMagnitude > getMaxTranslationSpeed()) {
                // double correctingFactor = getMaxTranslationSpeed() / velocityMagnitude;
                // velocityX *= correctingFactor;
                // velocityY *= correctingFactor;
                // }
                // if (desiredRotation.isPresent()) {
                // yield fieldCentricFacingAngleDrive
                // .withVelocityX(velocityX)
                // .withVelocityY(velocityY)
                // .withTargetDirection(desiredRotation.get());
                // }
                // yield fieldCentricDrive
                // .withVelocityX(velocityX)
                // .withVelocityY(velocityY);
                // }
            };
        }
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

    public void setDesiredRotation(Rotation2d desiredRotation) {
        this.externalDesiredRotation = Optional.of(desiredRotation);
    }

    public void clearDesiredRotation() {
        this.externalDesiredRotation = Optional.empty();
    }

    public double getMaxTranslationSpeed() {
        return kMaxTranslationSpeed
                * (getDriveFlagValue(DriveFlag.SLOW_MODE) ? kPrimarySlowModeTranslationFactor : 1);
    }

    public double getMaxRotationSpeed() {
        return kMaxRotationSpeed * (getDriveFlagValue(DriveFlag.SLOW_MODE) ? kPrimarySlowModeRotationFactor : 1);
    }

    public Translation2d getInputSpeedTranslation(boolean fieldRelative) {
        return getInputTranslation(fieldRelative).times(getMaxTranslationSpeed());
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
        if (externalDesiredRotation.isPresent()) {
            noInputRotation = getInputRotation().isEmpty();
        }
        return getInputTranslation(false).getNorm() == 0.0 && getInputRotationVelocity() == 0.0 && noInputRotation;
    }

    public boolean isAtDesiredRotation() {
        if (externalDesiredRotation.isEmpty()) {
            return false;
        }
        return Math.abs(getState().Pose.getRotation().minus(externalDesiredRotation.get()).getMeasure()
                .baseUnitMagnitude()) < kPrimaryAutoBrakeReachedDesiredAngleTolerance
                        .baseUnitMagnitude();
    }

    public boolean canAutoBrake() {
        boolean atDesiredRotation = (externalDesiredRotation.isEmpty() ? true : isAtDesiredRotation());
        return getDriveFlagValue(DriveFlag.AUTO_BRAKE) && isDriveIdle() && atDesiredRotation;
    }

    public boolean isCrashing() {
        double xyAccelerationMagnitude = Math.hypot(pigeon.getAccelerationX().getValue().in(MetersPerSecondPerSecond),
                pigeon.getAccelerationY().getValue().in(MetersPerSecondPerSecond));
        return xyAccelerationMagnitude >= 20.0;
    }

    public boolean isSlipping() {
        double expectedSpeed = Math.hypot(getState().Speeds.vxMetersPerSecond, getState().Speeds.vyMetersPerSecond);
        double actualSpeed = Math.hypot(estimatedRealChassisSpeeds.vxMetersPerSecond,
                estimatedRealChassisSpeeds.vyMetersPerSecond);
        return slippingBucketFilter.calculate(expectedSpeed - actualSpeed) > 2.0;
    }
}
