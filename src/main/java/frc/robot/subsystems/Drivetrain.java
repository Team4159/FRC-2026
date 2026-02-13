package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class Drivetrain extends CommandSwerveDrivetrain {
    private static final double kInputTranslationExponent = 2.0; 
    private static final double kInputRotationExponent = 2.0; 

    private final Supplier<Double> inputX;
    private final Supplier<Double> inputY;
    private final Supplier<Double> inputRotation;

    public Drivetrain(CommandXboxController controller) {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        this.inputX = () -> -controller.getLeftY();
        this.inputY = () -> -controller.getLeftX();
        this.inputRotation = () -> -controller.getRightX();
    }

    /** @return the field relative translation input (-left joystick y input, -left joystick x input), from magnitude range -1 to 1. a deadzone is applied.*/
    public Translation2d getInputTranslation() {
        Translation2d rawInput = new Translation2d(inputX.get(), inputY.get());
        Vector<N2> filteredInputVector = MathUtil.applyDeadband(rawInput.toVector(), OperatorConstants.kDriverControllerTranslationDeadband, 1);
        return new Translation2d(filteredInputVector);
    }

    /** @return the field relative x input (-left joystick y input), from range -1 to 1. a deadzone and quadratic are applied for better control.*/
    public double getInputX() {
        double input = getInputTranslation().getX();
        return Math.abs(Math.pow(input, kInputTranslationExponent)) * Math.signum(input);
    }

    /** @return the field relative y input (-left joystick x input), from range -1 to 1. a deadzone and quadratic are applied for better control.*/
    public double getInputY() {
        double input = getInputTranslation().getY();
        return Math.abs(Math.pow(input, kInputTranslationExponent)) * Math.signum(input);
    }

    /** @return the field relative rotation input (-right joystick x), from range -1 to 1. a deadzone and quadratic are applied for better control.*/
    public double getInputRotation() {
        double rawInput = inputRotation.get();
        double filteredInput = MathUtil.applyDeadband(Math.abs(rawInput), OperatorConstants.kDriverControllerRotationDeadband, 1);
        return Math.abs(Math.pow(filteredInput, kInputRotationExponent)) * Math.signum(rawInput);
    }
}
