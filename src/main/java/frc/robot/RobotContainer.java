// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.AlignConstants.TowerAlignGoal;
import frc.robot.Constants.OperatorConstants.DriveMode;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.HopperConstants.HopperState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Hopper;
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
    private final Trigger primaryAutoAimTrigger = primaryController.a();
    //Controllers
    private final Trigger primaryRightClimbAlignTrigger = primaryController.povRight();
    private final CommandXboxController secondary = new CommandXboxController(1);

    //Primary Triggers
    //private final Trigger primaryMiddleFrontClimbAlignTrigger = primaryController.povUp();
    //private final Trigger primaryMiddleBackClimbAlignTrigger = primaryController.povDown();

    //Secondary Triggers
    private final Trigger feedHopper = secondary.rightBumper();
    private final Trigger reverseFeederHopper = secondary.rightTrigger(0.1);

    private final Trigger intakeTrigger = secondary.leftBumper();
    private final Trigger stowIntakeTrigger = secondary.leftTrigger();

    private final Trigger shootTrigger = secondary.x();

    private final Trigger manualHoodIncrease = secondary.povUp();
    private final Trigger manualHoodDecrease = secondary.povDown();

    private final Trigger logData = secondary.a();

    //Subsystems
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Hopper hopper = new Hopper();
    private final LEDs leds = new LEDs();
    private final Drivetrain drivetrain = new Drivetrain(primaryController);
    private final PhotonVision photonVision = new PhotonVision(drivetrain);

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();
    private final ConfigurableAuto configurableAuto;

    public RobotContainer() {
        //set auto command for drivetrai
        drivetrain.setAutonomousAutoAimCommand(new AutoAim(drivetrain, shooter, hopper, leds, true));
        //Choreo Auto
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, drivetrain);

        configurableAuto = new ConfigurableAuto(autoFactory, drivetrain, shooter, null, hopper, leds);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        autoChooser.addRoutine("Left", autoRoutines::leftAuto);
        autoChooser.addRoutine("Right", autoRoutines::rightAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(drivetrain.new Drive(DriveMode.FREE,
                () -> primaryRobotRelativeTrigger.getAsBoolean()));
        primaryAutoAimTrigger.whileTrue(new AutoAim(drivetrain, shooter, hopper, leds, false));

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

        // Reset the field-centric heading on left bumper press.
        primaryZeroTrigger.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        feedHopper.whileTrue(new ParallelCommandGroup(shooter.new ChangeState(FeederState.FEED), hopper.new ChangeState(HopperState.FEED)));
        reverseFeederHopper.whileTrue(new ParallelCommandGroup(shooter.new ChangeState(FeederState.UNSTUCKFEEDER), hopper.new ChangeState(HopperState.REVERSE)));

        intakeTrigger.whileTrue(intake.new ChangeStates(IntakeState.DOWN_ON));
        stowIntakeTrigger.whileTrue(intake.new ChangeStates(IntakeState.STOP));

        shootTrigger.whileTrue(shooter.new ChangeVelocity(RPM.of(3200)));
        //.onFalse(shooter.new ChangeVelocity(RPM.of(0)));

        manualHoodIncrease.whileTrue(new InstantCommand(() -> shooter.incrementSetpoint(Degrees.of(5))));
        manualHoodDecrease.whileTrue(new InstantCommand(() -> shooter.incrementSetpoint(Degrees.of(-5))));

        logData.onTrue(new InstantCommand(() -> shooter.logData(drivetrain.getDistanceFromHub())));
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return configurableAuto.getRoutine().cmd();
    }
}