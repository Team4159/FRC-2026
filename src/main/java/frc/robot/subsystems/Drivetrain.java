package frc.robot.subsystems;

import static frc.robot.Constants.OperatorConstants.*;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.FieldUtil;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.DriveMode;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class Drivetrain extends CommandSwerveDrivetrain {

    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(15, 0, 0);
    private final SwerveRequest.SwerveDriveBrake brakeDrive = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt pointDrive = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.Idle idleDrive = new SwerveRequest.Idle();

    private final Supplier<Double> inputX;
    private final Supplier<Double> inputY;
    private final Supplier<Double> inputRotation;

    private boolean reduceSpeedEnabled = false;
    private boolean driveAssistEnabled = true;

    public Drivetrain(CommandXboxController controller) {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight,
                TunerConstants.BackLeft, TunerConstants.BackRight);
        this.inputX = () -> -controller.getLeftY();
        this.inputY = () -> -controller.getLeftX();
        this.inputRotation = () -> -controller.getRightX();
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

        private double getSpeedX() {
            return getInputX() * getMaxTranslationSpeed();
        }

        private double getSpeedY() {
            return getInputY() * getMaxTranslationSpeed();
        }

        private double getSpeedRotation() {
            return getInputRotation() * getMaxRotationSpeed();
        }

        private Optional<ChassisSpeeds> driveAssist() {
            if (!driveAssistEnabled || DriverStation.isTest()) {
                return Optional.empty();
            }
            Pose2d robotPose = getState().Pose;

            // trench assist
            var trenchZone = FieldUtil.getPoseTrenchZone(robotPose);
            if (trenchZone.isPresent()) {
                
                Distance errorX = trenchZone.get().x.minus(robotPose.getMeasureX());
                Distance errorY = trenchZone.get().y.minus(robotPose.getMeasureY());
                double inputFieldX = getInputX()
                        * (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) ? 1 : -1);

                boolean hasPassed = !errorX.isNear(Meters.zero(), OperatorConstants.kTrenchAssistPassPositionTolerance);
                boolean isNotApproaching = (Math.signum(inputFieldX) == -Math.signum(errorX.magnitude()))
                        || Math.abs(inputFieldX) <= OperatorConstants.kTrenchAssistPassInputTolerance;
                if (hasPassed && isNotApproaching) {
                    return Optional.empty();
                }

                double vy = -kMaxTranslationSpeed * Math.signum(errorY.magnitude()) * Math.abs(inputFieldX);
                double influence = OperatorConstants.kTrenchAssistAlignInfluence * getSpeedY();
                boolean insideTolerance = errorY.isNear(Meters.zero(), OperatorConstants.kTrenchAssistAlignPositionTolerance);
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
                    if (robotRelativeSupplier.getAsBoolean()) {
                        yield robotCentricDrive.withVelocityX(getSpeedX())
                                .withVelocityY(getSpeedY())
                                .withRotationalRate(getSpeedRotation());
                    }
                    var assistSpeed = driveAssist();
                    var velocityY = assistSpeed.isEmpty() ? getSpeedY()
                            : assistSpeed.get().vyMetersPerSecond;
                    yield fieldCentricDrive.withVelocityX(getSpeedX())
                            .withVelocityY(velocityY)
                            .withRotationalRate(getSpeedRotation());
                }
                case BRAKE -> brakeDrive;
                case POINT -> pointDrive
                        .withModuleDirection(new Rotation2d(getInputX(), getInputY()));
                case IDLE -> idleDrive;
                case ALIGN -> {
                    Translation2d rawInput = getRawInputTranslation();
                    double x = Math.signum(MathUtil.applyDeadband(rawInput.getX(), kPrimaryAlignModeDeadband, 1));
                    double y = Math.signum(MathUtil.applyDeadband(rawInput.getY(), kPrimaryAlignModeDeadband, 1));

                    if (x != 0 && y != 0) {
                        x /= Math.sqrt(2);
                        y /= Math.sqrt(2);
                    }

                    double velocityX = kPrimaryAlignModeSpeedTranslationFactor * x * getMaxTranslationSpeed();
                    double velocityY = kPrimaryAlignModeSpeedTranslationFactor * y * getMaxTranslationSpeed();
                    double rotationalRate = kPrimaryAlignModeSpeedRotationFactor * kMaxRotationSpeed * getInputRotation();

                    if (robotRelativeSupplier.getAsBoolean()) {
                        yield robotCentricDrive.withVelocityX(velocityX)
                                .withVelocityY(velocityY)
                                .withRotationalRate(rotationalRate);
                    }
                    yield fieldCentricDrive.withVelocityX(velocityX)
                            .withVelocityY(velocityY)
                            .withRotationalRate(rotationalRate);
                }
                case INTAKE -> {
                    // TODO: decide whats the optimal deadzone or have some heuristic to check if
                    // the joystick was released
                    if (Math.hypot(getInputX(), getInputY()) >= 0.2) {
                        yield fieldCentricFacingAngleDrive
                                .withVelocityX(getSpeedX())
                                .withVelocityY(getSpeedY())
                                .withTargetDirection(new Rotation2d(getInputX(), getInputY()));
                    }
                    yield fieldCentricDrive
                            .withVelocityX(getSpeedX())
                            .withVelocityY(getSpeedY());
                }
                case RADIAL -> {
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
                    yield fieldCentricDrive
                            .withVelocityX(kMaxTranslationSpeed
                                    * (-radialInput * radialVector.getX() + tangentialInput * tangentialVector.getX()))
                            .withVelocityY(kMaxTranslationSpeed
                                    * (-radialInput * radialVector.getY() - tangentialInput * tangentialVector.getY()));
                }
            };
        }
    }

    public void enableReduceSpeed(boolean enabled) {
        reduceSpeedEnabled = enabled;
    }

    public void enableDriveAssist(boolean enabled) {
        driveAssistEnabled = enabled;
    }

    /**
     * @return the alliance relative translation input (-left joystick y input,
     *         -left
     *         joystick x input), from magnitude range -1 to 1. no deadzone is
     *         applied
     */
    public Translation2d getRawInputTranslation() {
        return new Translation2d(inputX.get(), inputY.get());
    }

    /**
     * @return the alliance relative translation input (-left joystick y input,
     *         -left
     *         joystick x input), from magnitude range -1 to 1. a deadzone is
     *         applied.
     */
    public Translation2d getInputTranslation() {
        Translation2d rawInput = getRawInputTranslation();
        Vector<N2> filteredInputVector = rawInput.toVector();
        filteredInputVector = MathUtil.applyDeadband(filteredInputVector, kPrimaryTranslationDeadband, 1);
        filteredInputVector = filteredInputVector.div(kPrimaryTranslationRadius);
        if (filteredInputVector.norm() > 1) {
            filteredInputVector = filteredInputVector.div(filteredInputVector.norm());
        }
        return new Translation2d(filteredInputVector);
    }

    /**
     * @return the alliance relative x input (-left joystick y input), from range -1
     *         to
     *         1. a deadzone and quadratic are applied for better control.
     */
    public double getInputX() {
        // double input = getInputTranslation().getX();
        // return Math.abs(Math.pow(input, kPrimaryTranslationExponent)) *
        // Math.signum(input);
        return getInputTranslation().getX();
    }

    /**
     * @return the alliance relative y input (-left joystick x input), from range -1
     *         to
     *         1. a deadzone and quadratic are applied for better control.
     */
    public double getInputY() {
        // double input = getInputTranslation().getY();
        // return Math.abs(Math.pow(input, kPrimaryTranslationExponent)) *
        // Math.signum(input);
        return getInputTranslation().getY();
    }

    /**
     * @return the field relative rotation input (-right joystick x), from range -1
     *         to 1. no deadzone is applied
     */
    public double getRawInputRotation() {
        return inputRotation.get();
    }

    /**
     * @return the field relative rotation input (-right joystick x), from range -1
     *         to 1. a deadzone and quadratic are applied for better control.
     */
    public double getInputRotation() {
        double rawInput = getRawInputRotation();
        double filteredInput = MathUtil.applyDeadband(Math.abs(rawInput),
                kPrimaryRotationDeadband, 1);
        return Math.abs(Math.pow(filteredInput, kPrimaryRotationExponent)) * Math.signum(rawInput);
    }

}
