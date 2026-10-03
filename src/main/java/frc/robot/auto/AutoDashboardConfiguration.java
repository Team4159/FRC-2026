package frc.robot.auto;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Creates and publishes the controls consumed by Elastic's Auto dashboard tab. */
public final class AutoDashboardConfiguration {

    private static final String KEY_DIRECTORY = "Auto/";

    private static final double START_DELAY_DEFAULT = 0.0;

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

    public static Time getStartDelay() {
        return Seconds.of(SmartDashboard.getNumber(key("Start Delay"), START_DELAY_DEFAULT));
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
        SmartDashboard.setDefaultNumber(key("Start Delay"), START_DELAY_DEFAULT);
        SmartDashboard.putData(key("Side"), sideChooser);
        SmartDashboard.putData(key("Intake 1"), intakeChooser1);
        SmartDashboard.putData(key("Shoot 1"), shootChooser1);
        SmartDashboard.putData(key("Intake 2"), intakeChooser2);
        SmartDashboard.putData(key("Shoot 2"), shootChooser2);
        SmartDashboard.putData(key("Generate"), generateCommand);
        SmartDashboard.putData(key("Generated Routine Display"), generatedRoutineDisplay);
    }

    private static String key(String name) {
        return KEY_DIRECTORY + name;
    }
}
