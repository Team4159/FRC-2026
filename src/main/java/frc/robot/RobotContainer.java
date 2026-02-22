// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.DriveMode;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    private final Telemetry logger = new Telemetry();

    private final CommandXboxController primaryController = new CommandXboxController(
            OperatorConstants.kPrimaryControllerPort);
    private final Trigger primaryIntakeModeTrigger = primaryController.b();
    private final Trigger primaryRadialModeTrigger = primaryController.y();
    private final Trigger primaryRobotAlignModeTrigger = primaryController.leftBumper();
    private final Trigger primaryRobotRelativeTrigger = primaryController.leftTrigger();
    private final Trigger primaryReduceSpeedTrigger = primaryController.rightTrigger();
    private final Trigger primaryDriverAssistTrigger = primaryController.rightBumper();
    private final Trigger primaryZeroTrigger = primaryController.back();

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
        drivetrain.setDefaultCommand(drivetrain.new Drive(DriveMode.FREE,
                () -> primaryRobotRelativeTrigger.getAsBoolean()));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.new Drive(DriveMode.IDLE).ignoringDisable(true));

        // test mode
        primaryController.a().and(DriverStation::isTest).whileTrue(drivetrain.new Drive(DriveMode.BRAKE));
        primaryController.b().and(DriverStation::isTest).whileTrue(drivetrain.new Drive(DriveMode.POINT));

        // teleop mode
        primaryRobotAlignModeTrigger.and(DriverStation::isTeleop).whileTrue(drivetrain.new Drive(DriveMode.ALIGN,
                () -> primaryRobotRelativeTrigger.getAsBoolean()));
        primaryIntakeModeTrigger.and(DriverStation::isTeleop).whileTrue(drivetrain.new Drive(DriveMode.INTAKE));
        primaryRadialModeTrigger.and(DriverStation::isTeleop).whileTrue(drivetrain.new Drive(DriveMode.RADIAL));
        primaryReduceSpeedTrigger.and(DriverStation::isTeleop).onChange(
                Commands.runOnce(() -> drivetrain.enableReduceSpeed(primaryReduceSpeedTrigger.getAsBoolean())));
        primaryDriverAssistTrigger.and(DriverStation::isTeleop).onChange(
                Commands.runOnce(() -> drivetrain.enableDriveAssist(!primaryDriverAssistTrigger.getAsBoolean())));

        // Reset the field-centric heading on left bumper press.
        primaryZeroTrigger.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}