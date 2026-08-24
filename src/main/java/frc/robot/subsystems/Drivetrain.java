package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import java.util.function.Supplier;

public class Drivetrain extends CommandSwerveDrivetrain {

    public final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest.SwerveDriveBrake brakeDrive = new SwerveRequest.SwerveDriveBrake();
    public final SwerveRequest.Idle idleDrive = new SwerveRequest.Idle();

    private final Supplier<Double> inputX;
    private final Supplier<Double> inputY;
    private final Supplier<Double> inputRotation;

    /**
     * @param controller a CommandXboxController used to control the robot in teleop
     */
    public Drivetrain(CommandXboxController controller) {
        super(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        );
        this.inputX = () -> -controller.getLeftY();
        this.inputY = () -> -controller.getLeftX();
        this.inputRotation = () -> -controller.getRightX();
    }

    /** @return the field relative translation input (-left joystick y input, -left joystick x input), from magnitude range -1 to 1. a deadzone is applied.*/
    public Translation2d getInputTranslation() {
        Translation2d rawInput = new Translation2d(inputX.get(), inputY.get());
        Vector<N2> inputVector = MathUtil.applyDeadband(
            rawInput.toVector(),
            OperatorConstants.PRIMARY_TRANSLATION_DEADBAND,
            1
        );
        if (inputVector.norm() > 1.0) {
            inputVector = inputVector.unit();
        } else if (inputVector.norm() > 0.0) {
            inputVector = inputVector
                .unit()
                .times(Math.pow(inputVector.norm(), OperatorConstants.PRIMARY_TRANSLATION_EXPONENT));
        }
        return new Translation2d(inputVector);
    }

    /** @return the field relative x input (-left joystick y input), from range -1 to 1. a deadzone and quadratic are applied for better control.*/
    public double getInputX() {
        return getInputTranslation().getX();
    }

    /** @return the field relative y input (-left joystick x input), from range -1 to 1. a deadzone and quadratic are applied for better control.*/
    public double getInputY() {
        return getInputTranslation().getY();
    }

    /** @return the field relative rotation input (-right joystick x), from range -1 to 1. a deadzone and quadratic are applied for better control.*/
    public double getInputRotation() {
        double rawInput = inputRotation.get();
        double filteredInput = MathUtil.applyDeadband(
            Math.abs(rawInput),
            OperatorConstants.PRIMARY_ROTATION_DEADBAND,
            1
        );
        return Math.pow(filteredInput, OperatorConstants.PRIMARY_ROTATION_EXPONENT) * Math.signum(rawInput);
    }

    public Command drive() {
        return Commands.run(() -> {
            LinearVelocity x = DrivetrainConstants.MAX_TRANSLATION_SPEED.times(getInputX());
            LinearVelocity y = DrivetrainConstants.MAX_TRANSLATION_SPEED.times(getInputY());
            AngularVelocity rotation = DrivetrainConstants.MAX_ROTATION_SPEED.times(getInputRotation());
            if (x.magnitude() == 0.0 && y.magnitude() == 0.0 && rotation.magnitude() == 0.0) {
                setControl(brakeDrive);
                return;
            }
            setControl(fieldCentricDrive.withVelocityX(x).withVelocityY(y).withRotationalRate(rotation));
        }, this);
    }

    public Command idle() {
        return Commands.runOnce(() -> {
            setControl(idleDrive);
        }, this);
    }
}
