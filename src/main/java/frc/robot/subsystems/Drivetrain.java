package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class Drivetrain extends CommandSwerveDrivetrain {
    private static final double kInputXYExponent = 2.0; 
    private static final double kInputRotationExponent = 2.0; 

    private final CommandXboxController controller;

    public Drivetrain(CommandXboxController controller) {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        this.controller = controller;
    }

    /** @return the field relative x input (-left joystick y input), from range -1 to 1. a deadzone and quadratic are applied for better control.*/
    public double getInputX() {
        double rawInput = -controller.getLeftY();
        double filteredInput = MathUtil.applyDeadband(Math.abs(rawInput), OperatorConstants.kDriverControllerXYDeadband, 1);
        return Math.abs(Math.pow(filteredInput, kInputXYExponent)) * Math.signum(rawInput);
    }

    /** @return the field relative y input (-left joystick x input), from range -1 to 1. a deadzone and quadratic are applied for better control.*/
    public double getInputY() {
        double rawInput = -controller.getLeftX();
        double filteredInput = MathUtil.applyDeadband(Math.abs(rawInput), OperatorConstants.kDriverControllerXYDeadband, 1);
        return Math.abs(Math.pow(filteredInput, kInputXYExponent)) * Math.signum(rawInput);
    }

    /** @return the field relative rotation input (-right joystick x), from range -1 to 1. a deadzone and quadratic are applied for better control.*/
    public double getInputRotation() {
        double rawInput = -controller.getRightX();
        double filteredInput = MathUtil.applyDeadband(Math.abs(rawInput), OperatorConstants.kDriverControllerXYDeadband, 1);
        return Math.abs(Math.pow(filteredInput, kInputRotationExponent)) * Math.signum(rawInput);
    }
}
