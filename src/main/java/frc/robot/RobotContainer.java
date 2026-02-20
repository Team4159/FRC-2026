// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAim;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DriveMode;

public class RobotContainer {
    private final Telemetry logger = new Telemetry();

    private final CommandXboxController primaryController = new CommandXboxController(0);
    private final Trigger AutoAimTrigger = primaryController.rightBumper();

    public final Drivetrain drivetrain = new Drivetrain(primaryController);

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        autoChooser.addRoutine("Left", autoRoutines::leftAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.drive(DriveMode.FIELD_CENTRIC)
        );

        AutoAimTrigger.whileTrue(new AutoAim(drivetrain));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.drive(DriveMode.IDLE).ignoringDisable(true));

        primaryController.leftStick().whileTrue(drivetrain.drive(DriveMode.ROBOT_CENTRIC));
        primaryController.x().whileTrue(drivetrain.drive(DriveMode.INTAKE));
        primaryController.y().whileTrue(drivetrain.drive(DriveMode.SHOOT));
        if (DriverStation.isTest()) {
            primaryController.a().whileTrue(drivetrain.drive(DriveMode.BRAKE));
            primaryController.b().whileTrue(drivetrain.drive(DriveMode.POINT));
        }

        // Reset the field-centric heading on left bumper press.
        primaryController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}