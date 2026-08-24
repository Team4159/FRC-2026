// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

    private final Telemetry logger = new Telemetry();

    // Controllers
    private final CommandXboxController primaryController = new CommandXboxController(0);

    // Subsystems
    private final Drivetrain drivetrain = new Drivetrain(primaryController);

    // Triggers
    private final Trigger drivetrainZeroTrigger = primaryController.back();

    public RobotContainer() {
        drivetrain.registerTelemetry(logger::telemeterize);
        drivetrain.setDefaultCommand(drivetrain.drive());
        RobotModeTriggers.disabled().whileTrue(drivetrain.idle().ignoringDisable(true));
        configureBindings();
    }

    private void configureBindings() {
        drivetrainZeroTrigger.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return Commands.none();
    }
}
