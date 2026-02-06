package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class Drivetrain extends CommandSwerveDrivetrain {
    private final CommandXboxController controller;

    public Drivetrain(CommandXboxController controller) {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        this.controller = controller;
    }

    public double getInputX() {
        double rawInput = -controller.getLeftY();
        double filteredInput = MathUtil.applyDeadband(rawInput, OperatorConstants.kDriverControllerVelocityDeadband);
        return Math.abs(Math.pow(filteredInput, 2)) * Math.signum(filteredInput);
    }

    public double getInputY() {
        double rawInput = -controller.getLeftX();
        double filteredInput = MathUtil.applyDeadband(rawInput, OperatorConstants.kDriverControllerVelocityDeadband);
        return Math.abs(Math.pow(filteredInput, 2)) * Math.signum(filteredInput);
    }

    public double getInputRotation() {
        double rawInput = -controller.getRightX();
        double filteredInput = MathUtil.applyDeadband(rawInput, OperatorConstants.kDriverControllerRotationDeadband, rawInput);
        return Math.abs(Math.pow(filteredInput, 2)) * Math.signum(filteredInput);
    }
}
