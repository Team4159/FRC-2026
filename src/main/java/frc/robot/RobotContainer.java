// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.FluentTrigger;
import frc.lib.HIDRumble;
import frc.lib.HIDRumble.RumbleRequest;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.AlignConstants.TowerAlignGoal;
import frc.robot.Constants.OperatorConstants.DriveFlag;
import frc.robot.Constants.OperatorConstants.DriveMode;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.HopperConstants.HopperState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoLob;
import frc.robot.commands.HubShoot;
import frc.robot.commands.TowerShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private final CommandXboxController primaryController = new CommandXboxController(
            OperatorConstants.kPrimaryControllerPort);
    private final CommandXboxController secondaryController = new CommandXboxController(
            OperatorConstants.kSecondaryControllerPort);
    private final Trigger primaryZeroTrigger = primaryController.back();
    private final Trigger primaryIntakeAssistTrigger = primaryController.rightBumper();
    // private final Trigger primaryRadialModeTrigger = primaryController.y();
    private final Trigger primaryRobotManualAlignModeTrigger = primaryController.leftBumper();
    private final Trigger primaryRobotRelativeTrigger = primaryController.leftTrigger(0.1);
    private final Trigger primaryReduceSpeedTrigger = primaryController.rightTrigger();
    private final Trigger primaryDriverAssistTrigger = primaryController.start();
    private final Trigger primaryLeftClimbAlignTrigger = primaryController.povLeft();
    private final Trigger primaryAutoAimTrigger = primaryController.x();
    private final Trigger primaryAutoLobTrigger = primaryController.a();
    private final Trigger primaryRightClimbAlignTrigger = primaryController.povRight();

    // Secondary Triggers
    private final Trigger feedHopperTrigger = secondaryController.rightBumper();
    private final Trigger reverseFeederHopperTrigger = secondaryController.rightTrigger(0.1);

    private final Trigger manualHoodPitchUpTrigger = secondaryController.povUp();
    private final Trigger manualHoodPitchDownTrigger = secondaryController.povDown();

    private final Trigger intakeTrigger = secondaryController.leftBumper();
    private final Trigger outtakeTrigger = secondaryController.leftTrigger(0.1);
    private final Trigger compressIntakeTrigger = secondaryController.a();
    private final Trigger bounceIntakeTrigger = secondaryController.b();

    //private final Trigger raiseClimbTrigger = secondaryController.povLeft();
    //private final Trigger lowerClimbTrigger = secondaryController.povRight();

    private final Trigger shootTrigger = secondaryController.povRight();

    private final Trigger hubShootTrigger = secondaryController.x();
    private final Trigger towerShootTrigger = secondaryController.y();

    // Subsystems
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Hopper hopper = new Hopper();
    private final LEDs leds = new LEDs();
    private final Drivetrain drivetrain = new Drivetrain(primaryController);
    @SuppressWarnings("unused")
    //periodic function inside photon vision class used to send vision data
    private final PhotonVision photonVision = new PhotonVision(drivetrain);

    /* Path follower */
    private final AutoFactory autoFactory;
    private final ConfigurableAuto configurableAuto;

    private final Telemetry logger = new Telemetry();

    public RobotContainer() {
        // set auto command for drivetrain
        drivetrain.setAutonomousAutoAimCommand(new AutoAim(drivetrain, shooter, hopper, intake, leds, true, Optional.empty()));
        // Choreo Auto
        autoFactory = drivetrain.createAutoFactory();

        configurableAuto = new ConfigurableAuto(autoFactory, drivetrain, shooter, intake, hopper, leds);

        drivetrain.crashTrigger.onTrue(Commands
                .runOnce(() -> HIDRumble.rumble(primaryController.getHID(),
                        new RumbleRequest(RumbleType.kRightRumble, 1, 0.5, 1))));

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(drivetrain.new Drive(DriveMode.TELEOP,
                primaryRobotRelativeTrigger::getAsBoolean));
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.new Drive(DriveMode.IDLE).ignoringDisable(true));

        // test mode
        primaryController.a().and(DriverStation::isTest).whileTrue(drivetrain.new Drive(DriveMode.BRAKE));
        primaryController.b().and(DriverStation::isTest).whileTrue(drivetrain.new Drive(DriveMode.POINT));

        // teleop mode
        primaryReduceSpeedTrigger.and(DriverStation::isTeleop).onChange(
                Commands.runOnce(() -> drivetrain.setDriveFlagValue(DriveFlag.SLOW_MODE,
                        primaryReduceSpeedTrigger.getAsBoolean())));
        primaryDriverAssistTrigger.and(DriverStation::isTeleop).onTrue(
                Commands.runOnce(() -> {
                    HIDRumble.rumble(primaryController.getHID(), new RumbleRequest(RumbleType.kLeftRumble, 0.5, 0.25));
                    drivetrain.setDriveFlagValue(DriveFlag.DRIVE_ASSIST,
                            !drivetrain.getDriveFlagValue(DriveFlag.DRIVE_ASSIST));
                }));
        primaryRobotManualAlignModeTrigger.and(DriverStation::isTeleop).onChange(
                Commands.runOnce(() -> {drivetrain.setDriveFlagValue(DriveFlag.MANUAL_ALIGN,
                        primaryRobotManualAlignModeTrigger.getAsBoolean());
                    System.out.println(primaryRobotManualAlignModeTrigger.getAsBoolean());}));
        primaryIntakeAssistTrigger.and(DriverStation::isTeleop).onChange(
                Commands.runOnce(() -> drivetrain.setDriveFlagValue(DriveFlag.INTAKE_ASSIST,
                        primaryIntakeAssistTrigger.getAsBoolean())));
        FluentTrigger.build()
                .bind(1, primaryLeftClimbAlignTrigger.and(DriverStation::isTeleop),
                        new AutoAlign(drivetrain, TowerAlignGoal.LEFT))
                .bind(1, primaryRightClimbAlignTrigger.and(DriverStation::isTeleop),
                        new AutoAlign(drivetrain, TowerAlignGoal.RIGHT))
                .bind(0, primaryAutoAimTrigger.and(DriverStation::isTeleop),
                        new AutoAim(drivetrain, shooter, hopper, intake, leds, false, Optional.of(primaryController)))
                .bind(0, primaryAutoLobTrigger.and(DriverStation::isTeleop),
                        new AutoLob(drivetrain, shooter, hopper, intake, leds, false));

        // Reset the field-centric heading on left bumper press.
        primaryZeroTrigger.onTrue(Commands.runOnce(() -> {
            HIDRumble.rumble(primaryController.getHID(), new RumbleRequest(RumbleType.kLeftRumble, 0.5, 0.25));
            drivetrain.seedFieldCentric();
        }));

        drivetrain.registerTelemetry(logger::telemeterize);

        feedHopperTrigger.whileTrue(new ParallelCommandGroup(shooter.new ChangeState(FeederState.FEED),
                hopper.new ChangeState(HopperState.FEED)));
        reverseFeederHopperTrigger.whileTrue(new ParallelCommandGroup(
                shooter.new ChangeState(FeederState.UNSTUCKFEEDER), hopper.new ChangeState(HopperState.REVERSE)));

        // intake
        intakeTrigger.whileTrue(new ParallelCommandGroup(intake.new ChangeStates(IntakeState.DOWN_ON),
                hopper.new ChangeState(HopperState.FEED)));//.onFalse(intake.new ChangeStates(IntakeState.BOUNCE_UP));
        outtakeTrigger.whileTrue(new ParallelCommandGroup(intake.new ChangeStates(IntakeState.DOWN_REV),
                hopper.new ChangeState((HopperState.REVERSE)), shooter.new ChangeState(FeederState.UNSTUCKFEEDER)));
        compressIntakeTrigger.whileTrue(intake.new ChangeStates(IntakeState.UP_OFF));
        bounceIntakeTrigger.whileTrue(intake.new BounceIntake());

        // raiseClimbTrigger.whileTrue(climber.new ChangeState(ClimberState.CLIMB));
        // lowerClimbTrigger.whileTrue(climber.new ChangeState(ClimberState.DOWN));

        manualHoodPitchDownTrigger.onTrue(new InstantCommand(() -> shooter.manualHood(5)));
        manualHoodPitchUpTrigger.onTrue(new InstantCommand(() -> shooter.manualHood(-5)));

        shootTrigger.whileTrue(shooter.new ChangeVelocity(ShooterConstants.shooterAngularVelocity));

        hubShootTrigger.whileTrue(new HubShoot(shooter, intake, hopper));
        towerShootTrigger.whileTrue(new TowerShoot(shooter, intake, hopper));
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return configurableAuto.getRoutine().cmd();
    }
}