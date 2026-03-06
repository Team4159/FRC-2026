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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.FieldUtil;
import frc.lib.HIDRumble;
import frc.lib.HIDRumble.RumbleRequest;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.DriveMode;
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

    private Optional<Rotation2d> desiredRotation = Optional.empty();

    public Drivetrain(CommandXboxController controller) {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight,
                TunerConstants.BackLeft, TunerConstants.BackRight);
        this.inputX = () -> -controller.getLeftY();
        this.inputY = () -> -controller.getLeftX();
        this.inputRotationVelocity = () -> -controller.getRightX();
        this.inputRotationX = () -> -controller.getRightY();
        this.inputRotationY = () -> -controller.getRightX();

        crashTrigger.onTrue(Commands
                .runOnce(() -> HIDRumble.rumble(controller.getHID(),
                        new RumbleRequest(RumbleType.kRightRumble, 1, 0.5, 1))));
    }

    public class Drive extends Command {
        private final BooleanSupplier robotRelativeSupplier;
        private final Supplier<SwerveRequest> driveSupplier;

        public Drive(DriveMode driveMode, BooleanSupplier robotRelativeSupplier) {
            this.robotRelativeSupplier = robotRelativeSupplier;
            this.driveSupplier = getDriveSupplier(driveMode);
            addRequirements(Drivetrain.this);
        }

        public Drive(DriveMode mode) {
            this(mode, () -> false);
        }

        @Override
        public void execute() {
            setControl(driveSupplier.get());
        }

        private double getMaxTranslationSpeed() {
            return kMaxTranslationSpeed * (reduceSpeedEnabled ? kPrimaryReduceSpeedTranslationFactor : 1);
        }

        private double getMaxRotationSpeed() {
            return kMaxRotationSpeed * (reduceSpeedEnabled ? kPrimaryReduceSpeedRotationFactor : 1);
        }

        private double getSpeedX(boolean fieldRelative) {
            return getInputX(fieldRelative) * getMaxTranslationSpeed();
        }

        private double getSpeedY(boolean fieldRelative) {
            return getInputY(fieldRelative) * getMaxTranslationSpeed();
        }

        private double getSpeedRotation() {
            return getInputRotationVelocity() * getMaxRotationSpeed();
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
                double influence = OperatorConstants.kTrenchAssistAlignInfluence * getSpeedY(true);
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
                    if (isInputIdle()) {
                        if (desiredRotation.isPresent()) {
                            if (Math.abs(Math
                                    .abs(getState().Pose.getRotation().minus(desiredRotation.get())
                                            .getDegrees())) < 1.0) {
                                yield brakeDrive;
                            }
                        } else {
                            yield brakeDrive;
                        }
                    }
                    if (robotRelativeSupplier.getAsBoolean()) {
                        if (desiredRotation.isPresent()) {
                            yield robotCentricFacingAngleDrive.withVelocityX(getSpeedX(true))
                                    .withVelocityY(getSpeedY(false))
                                    .withTargetDirection(desiredRotation.get());
                        }
                        yield robotCentricDrive.withVelocityX(getSpeedX(false))
                                .withVelocityY(getSpeedY(false))
                                .withRotationalRate(getSpeedRotation());
                    }
                    var assistSpeed = driveAssist();
                    var velocityY = assistSpeed.isEmpty() ? getSpeedY(true)
                            : assistSpeed.get().vyMetersPerSecond;
                    if (desiredRotation.isPresent()) {
                        yield fieldCentricFacingAngleDrive.withVelocityX(getSpeedX(true))
                                .withVelocityY(velocityY)
                                .withTargetDirection(desiredRotation.get());
                    }
                    yield fieldCentricDrive.withVelocityX(getSpeedX(true))
                            .withVelocityY(velocityY)
                            .withRotationalRate(getSpeedRotation());
                }
                case BRAKE -> brakeDrive;
                case POINT -> pointDrive
                        .withModuleDirection(new Rotation2d(getInputX(false), getInputY(false)));
                case IDLE -> idleDrive;
                case MANUAL_ALIGN -> {
                    Translation2d rawInput = getRawInputTranslation(!robotRelativeSupplier.getAsBoolean());
                    double x = Math.signum(MathUtil.applyDeadband(rawInput.getX(), kPrimaryAlignModeDeadband, 1));
                    double y = Math.signum(MathUtil.applyDeadband(rawInput.getY(), kPrimaryAlignModeDeadband, 1));

                    if (x != 0 && y != 0) {
                        x /= Math.sqrt(2);
                        y /= Math.sqrt(2);
                    }

                    double velocityX = kPrimaryAlignModeSpeedTranslationFactor * x * getMaxTranslationSpeed();
                    double velocityY = kPrimaryAlignModeSpeedTranslationFactor * y * getMaxTranslationSpeed();
                    double rotationalRate = kPrimaryAlignModeSpeedRotationFactor * kMaxRotationSpeed
                            * getInputRotationVelocity();

                    if (robotRelativeSupplier.getAsBoolean()) {
                        yield robotCentricDrive.withVelocityX(velocityX)
                                .withVelocityY(velocityY)
                                .withRotationalRate(rotationalRate);
                    }
                    yield fieldCentricDrive.withVelocityX(velocityX)
                            .withVelocityY(velocityY)
                            .withRotationalRate(rotationalRate);
                }
                case INTAKE_LEFT -> {
                    // TODO: decide whats the optimal deadzone or have some heuristic to check if
                    // the joystick was released
                    if (Math.hypot(getInputX(true), getInputY(true)) >= 0.2) {
                        Rotation2d rotation = new Rotation2d(getInputX(true), getInputY(true)).plus(new Rotation2d(Degrees.of(45.0)));
                        yield fieldCentricFacingAngleDrive
                                .withVelocityX(getSpeedX(true))
                                .withVelocityY(getSpeedY(true))
                                .withTargetDirection(rotation);
                    }
                    yield fieldCentricDrive
                            .withVelocityX(getSpeedX(true))
                            .withVelocityY(getSpeedY(true));
                }
                case INTAKE_RIGHT -> {
                    if (Math.hypot(getInputX(true), getInputY(true)) >= 0.2) {
                        Rotation2d rotation = new Rotation2d(getInputX(true), getInputY(true)).minus(new Rotation2d(Degrees.of(45.0)));
                        yield fieldCentricFacingAngleDrive
                                .withVelocityX(getSpeedX(true))
                                .withVelocityY(getSpeedY(true))
                                .withTargetDirection(rotation);
                    }
                    yield fieldCentricDrive
                            .withVelocityX(getSpeedX(true))
                            .withVelocityY(getSpeedY(true));
                }
                case RADIAL -> {
                    if (isInputIdle()) {
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

    public void setDesiredRotation(Rotation2d desiredRotation) {
        this.desiredRotation = Optional.of(desiredRotation);
    }

    public void clearDesiredRotation() {
        this.desiredRotation = Optional.empty();
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
            filteredInputVector = filteredInputVector.unit().times(Math.pow(filteredInputVector.norm(), kPrimaryTranslationExponent));
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
     * @return {@Code true} if there is no joystick input
     */
    public boolean isInputIdle() {
        boolean noInputRotation = true;
        if (desiredRotation.isPresent()) {
            noInputRotation = getInputRotation().isEmpty();
        }
        return getInputTranslation(false).getNorm() == 0.0 && getInputRotationVelocity() == 0.0 && noInputRotation;
    }

}
