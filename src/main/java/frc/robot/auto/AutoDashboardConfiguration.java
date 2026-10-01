package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Creates and publishes the controls consumed by Elastic's Auto dashboard tab. */
public final class AutoDashboardConfiguration {

    private AutoDashboardConfiguration() {}

    public static SendableChooser<String> sideChooser() {
        SendableChooser<String> chooser = new SendableChooser<>();
        chooser.addOption("Left", "L");
        chooser.addOption("Right", "R");
        chooser.addOption("Mid", "M");
        chooser.setDefaultOption("None", "None");
        return chooser;
    }

    public static SendableChooser<String> intakeChooser() {
        SendableChooser<String> chooser = new SendableChooser<>();
        chooser.addOption("Line", "LineIntake");
        chooser.addOption("Far", "FarIntake");
        chooser.addOption("Mid", "MidIntake");
        chooser.addOption("Close", "CloseIntake");
        chooser.addOption("Outer Sweep", "OuterIntake");
        chooser.addOption("Inner Sweep", "InnerIntake");
        chooser.setDefaultOption("None", "None");
        return chooser;
    }

    public static SendableChooser<String> shootChooser() {
        SendableChooser<String> chooser = new SendableChooser<>();
        chooser.addOption("Shoot", "Shoot");
        chooser.setDefaultOption("None", "None");
        return chooser;
    }

    public static void publish(
        SendableChooser<String> sideChooser,
        SendableChooser<String> intakeChooser1,
        SendableChooser<String> shootChooser1,
        SendableChooser<String> intakeChooser2,
        SendableChooser<String> shootChooser2,
        Command generateCommand,
        Field2d generatedRoutineDisplay
    ) {
        SmartDashboard.putData("Auto/Side", sideChooser);
        SmartDashboard.putData("Auto/Intake 1", intakeChooser1);
        SmartDashboard.putData("Auto/Shoot 1", shootChooser1);
        SmartDashboard.putData("Auto/Intake 2", intakeChooser2);
        SmartDashboard.putData("Auto/Shoot 2", shootChooser2);
        SmartDashboard.putData("Auto/Generate", generateCommand);
        SmartDashboard.putData("Auto/Generated Routine Display", generatedRoutineDisplay);
    }
}
