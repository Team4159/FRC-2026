// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.AlignConstants.TowerAlignGoal;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.OperatorConstants.DriveMode;
import frc.robot.commands.AutoAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private final Telemetry logger = new Telemetry();

    private final CommandXboxController primaryController = new CommandXboxController(
            OperatorConstants.kPrimaryControllerPort);
    private final Trigger primaryZeroTrigger = primaryController.back();
    private final Trigger primaryIntakeModeTrigger = primaryController.b();
    private final Trigger primaryRadialModeTrigger = primaryController.y();
    private final Trigger primaryRobotManualAlignModeTrigger = primaryController.leftBumper();
    private final Trigger primaryRobotRelativeTrigger = primaryController.leftTrigger();
    private final Trigger primaryReduceSpeedTrigger = primaryController.rightTrigger();
    private final Trigger primaryDriverAssistTrigger = primaryController.rightBumper();
    private final Trigger primaryLeftClimbAlignTrigger = primaryController.povLeft();
    private final Trigger primaryRightClimbAlignTrigger = primaryController.povRight();
    //private final Trigger primaryMiddleFrontClimbAlignTrigger = primaryController.povUp();
    //private final Trigger primaryMiddleBackClimbAlignTrigger = primaryController.povDown();

    private final CommandXboxController secondaryController = new CommandXboxController(1);
    private final Trigger secondarySpinTrigger = secondaryController.rightBumper();
    private final Trigger secondaryKickTrigger = secondaryController.rightTrigger();
    private final Trigger secondaryIntakeTrigger = secondaryController.leftTrigger();


    public final Drivetrain drivetrain = new Drivetrain(primaryController);
    public final Shooter shooter = new Shooter();
    public final Intake intake = new Intake();

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
        primaryRobotManualAlignModeTrigger.and(DriverStation::isTeleop).whileTrue(drivetrain.new Drive(DriveMode.MANUAL_ALIGN,
                () -> primaryRobotRelativeTrigger.getAsBoolean()));
        primaryIntakeModeTrigger.and(DriverStation::isTeleop).whileTrue(drivetrain.new Drive(DriveMode.INTAKE));
        primaryRadialModeTrigger.and(DriverStation::isTeleop).whileTrue(drivetrain.new Drive(DriveMode.RADIAL));
        primaryReduceSpeedTrigger.and(DriverStation::isTeleop).onChange(
                Commands.runOnce(() -> drivetrain.enableReduceSpeed(primaryReduceSpeedTrigger.getAsBoolean())));
        primaryDriverAssistTrigger.and(DriverStation::isTeleop).onChange(
                Commands.runOnce(() -> drivetrain.enableDriveAssist(!primaryDriverAssistTrigger.getAsBoolean())));
        primaryLeftClimbAlignTrigger.and(DriverStation::isTeleop).onTrue(new AutoAlign(drivetrain, TowerAlignGoal.LEFT, primaryRobotRelativeTrigger));
        primaryRightClimbAlignTrigger.and(DriverStation::isTeleop).onTrue(new AutoAlign(drivetrain, TowerAlignGoal.RIGHT, primaryRobotRelativeTrigger));
        //primaryMiddleFrontClimbAlignTrigger.and(DriverStation::isTeleop).onTrue(new AutoAlign(drivetrain, TowerAlignGoal.MIDDLE_FRONT, primaryRobotRelativeTrigger));
        //primaryMiddleBackClimbAlignTrigger.and(DriverStation::isTeleop).onTrue(new AutoAlign(drivetrain, TowerAlignGoal.MIDDLE_BACK, primaryRobotRelativeTrigger));

        secondarySpinTrigger.and(DriverStation::isTeleop).whileTrue(shooter.new Shoot(() -> RPM.of(3000.0)));
        secondaryKickTrigger.and(DriverStation::isTeleop).onTrue(Commands.runOnce(() -> shooter.setKickerSpeed(1)));
        secondaryKickTrigger.and(DriverStation::isTeleop).onFalse(Commands.runOnce(() -> shooter.setKickerSpeed(0)));

        intake.setDefaultCommand(intake.new ChangeStates(IntakeState.UP_OFF));
        secondaryIntakeTrigger.and(DriverStation::isTeleop).onTrue(intake.new ChangeStates(IntakeState.DOWN_ON));

        // Reset the field-centric heading on left bumper press.
        primaryZeroTrigger.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}