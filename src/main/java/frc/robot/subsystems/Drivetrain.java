package frc.robot.subsystems;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
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

    public Drivetrain(CommandXboxController controller) {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight,
                TunerConstants.BackLeft, TunerConstants.BackRight);
        this.inputX = () -> -controller.getLeftY();
        this.inputY = () -> -controller.getLeftX();
        this.inputRotation = () -> -controller.getRightX();
    }

    public Translation2d getRawInputTranslation() {
        return new Translation2d(inputX.get(), inputY.get());
    }

    /**
     * @return the field relative translation input (-left joystick y input, -left
     *         joystick x input), from magnitude range -1 to 1. a deadzone is
     *         applied.
     */
    public Translation2d getInputTranslation() {
        Translation2d rawInput = getRawInputTranslation();
        Vector<N2> filteredInputVector = MathUtil.applyDeadband(rawInput.toVector(),
                kDriverControllerTranslationDeadband, 1);
        return new Translation2d(filteredInputVector);
    }

    /**
     * @return the field relative x input (-left joystick y input), from range -1 to
     *         1. a deadzone and quadratic are applied for better control.
     */
    public double getInputX() {
        double input = getInputTranslation().getX();
        return Math.abs(Math.pow(input, kInputTranslationExponent)) * Math.signum(input);
    }

    /**
     * @return the field relative y input (-left joystick x input), from range -1 to
     *         1. a deadzone and quadratic are applied for better control.
     */
    public double getInputY() {
        double input = getInputTranslation().getY();
        return Math.abs(Math.pow(input, kInputTranslationExponent)) * Math.signum(input);
    }

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
                kDriverControllerRotationDeadband, 1);
        return Math.abs(Math.pow(filteredInput, kInputRotationExponent)) * Math.signum(rawInput);
    }

    public Command drive(DriveMode mode) {
        return applyRequest(() -> {
            return switch (mode) {
                case FIELD_CENTRIC -> fieldCentricDrive.withVelocityX(getInputX() * kMaxSpeed)
                        .withVelocityY(getInputY() * kMaxSpeed)
                        .withRotationalRate(getInputRotation() * kMaxAngularRate);
                case ROBOT_CENTRIC -> robotCentricDrive.withVelocityX(getInputX() * kMaxSpeed)
                        .withVelocityY(getInputY() * kMaxSpeed)
                        .withRotationalRate(getInputRotation() * kMaxAngularRate);
                case BRAKE -> brakeDrive;
                case POINT -> pointDrive
                        .withModuleDirection(new Rotation2d(getInputX(), getInputY()));
                case IDLE -> idleDrive;
                case INTAKE -> {
                    // TODO: decide whats the optimal deadzone or have some heuristic to check if
                    // the joystick was released
                    if (Math.hypot(getInputX(), getInputY()) >= 0.0) {
                        yield fieldCentricFacingAngleDrive
                                .withVelocityX(getInputX() * kMaxSpeed)
                                .withVelocityY(getInputY() * kMaxSpeed)
                                .withTargetDirection(new Rotation2d(getInputX(), getInputY()));
                    }
                    yield fieldCentricDrive
                            .withVelocityX(getInputX() * kMaxSpeed)
                            .withVelocityY(getInputY() * kMaxSpeed);
                }
                case SHOOT -> {
                    double radialInput = MathUtil.applyDeadband(inputX.get(), 0.25, 1);
                    double tangentialInput = MathUtil.applyDeadband(inputY.get(), 0.25, 1);
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
                            .withVelocityX(kMaxSpeed
                                    * (-radialInput * radialVector.getX() + tangentialInput * tangentialVector.getX()))
                            .withVelocityY(kMaxSpeed
                                    * (-radialInput * radialVector.getY() - tangentialInput * tangentialVector.getY()));
                }
            };
        });
    }

}
