// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.Constants.AlignConstants.TowerAlignGoal;
import frc.robot.Constants.OperatorConstants.DriveFlag;
import frc.robot.Constants.OperatorConstants.DriveMode;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.HopperConstants.HopperState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoBeachRecovery;
import frc.robot.commands.AutoLob;
import frc.robot.commands.HubShoot;
import frc.robot.commands.TowerShoot;
import frc.robot.commands.AutoBeachRecovery.BeachRecoveryMode;
import frc.robot.commands.AutoBeachRecovery.BeachRecoverySide;
import frc.robot.subsystems.Climber;
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
    private final Trigger primaryRobotRelativeTrigger = primaryController.leftTrigger();
    private final Trigger primarySlowModeTrigger = primaryController.rightTrigger();
    private final Trigger primaryDriveAssistTrigger = primaryController.start();
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

    // private final Trigger raiseClimbTrigger = secondaryController.povLeft();
    // private final Trigger lowerClimbTrigger = secondaryController.povRight();

    private final Trigger shootTrigger = secondaryController.povRight();

    private final Trigger hubShootTrigger = secondaryController.x();
    private final Trigger towerShootTrigger = secondaryController.y();

    // Subsystems
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Hopper hopper = new Hopper();
    private final LEDs leds = new LEDs();
    private final Climber climber = new Climber();
    private final Drivetrain drivetrain = new Drivetrain(primaryController);
    private final PhotonVision photonVision = new PhotonVision(drivetrain);

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();
    private final ConfigurableAuto configurableAuto;

    private final Telemetry logger = new Telemetry();

    public RobotContainer() {
        // set auto command for drivetrain
        drivetrain.setAutonomousAutoAimCommand(
                new AutoAim(drivetrain, shooter, hopper, intake, leds, true, Optional.empty()));
        // Choreo Auto
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, drivetrain);

        configurableAuto = new ConfigurableAuto(autoFactory, drivetrain, shooter, intake, hopper, leds);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        autoChooser.addRoutine("Left", autoRoutines::leftAuto);
        autoChooser.addRoutine("Right", autoRoutines::rightAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

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
        // primaryController.a().and(DriverStation::isTest).whileTrue(drivetrain.new Drive(DriveMode.BRAKE));
        primaryController.b().and(DriverStation::isTest).whileTrue(drivetrain.new Drive(DriveMode.POINT));
        primaryController.a().and(DriverStation::isTest).whileTrue(new AutoBeachRecovery(drivetrain, intake, BeachRecoveryMode.ZIG_ZAG, BeachRecoverySide.LEFT, new AutoAim(drivetrain, shooter, hopper, intake, leds, false, Optional.empty())));

        // teleop mode
        primarySlowModeTrigger.and(DriverStation::isTeleop)
                .whileTrue(drivetrain.new DriveFlagToggler(DriveFlag.SLOW_MODE));
        primaryDriveAssistTrigger.and(DriverStation::isTeleop).onTrue(
                Commands.runOnce(() -> {
                    HIDRumble.rumble(primaryController.getHID(),
                            new RumbleRequest(RumbleType.kLeftRumble, 0.5, 0.25));
                    drivetrain.setDriveFlagValue(DriveFlag.DRIVE_ASSIST,
                            !drivetrain.getDriveFlagValue(DriveFlag.DRIVE_ASSIST));
                }));
        primaryRobotManualAlignModeTrigger.and(DriverStation::isTeleop)
                .whileTrue(drivetrain.new DriveFlagToggler(DriveFlag.MANUAL_ALIGN));
        primaryIntakeAssistTrigger.and(DriverStation::isTeleop)
                .whileTrue(drivetrain.new DriveFlagToggler(DriveFlag.INTAKE_ASSIST));
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
                hopper.new ChangeState(HopperState.FEED))).onFalse(intake.new ChangeStates(IntakeState.BOUNCE_UP));
        outtakeTrigger.whileTrue(new ParallelCommandGroup(intake.new ChangeStates(IntakeState.DOWN_REV),
                hopper.new ChangeState((HopperState.REVERSE)), shooter.new ChangeState(FeederState.UNSTUCKFEEDER)));
        compressIntakeTrigger.whileTrue(intake.new ChangeStates(IntakeState.UP_OFF));
        bounceIntakeTrigger.whileTrue(intake.new BounceIntake());

        // raiseClimbTrigger.whileTrue(climber.new ChangeState(ClimberState.CLIMB));
        // lowerClimbTrigger.whileTrue(climber.new ChangeState(ClimberState.DOWN));

        manualHoodPitchDownTrigger.onTrue(new InstantCommand(() -> shooter.manualHood(5)));
        manualHoodPitchUpTrigger.onTrue(new InstantCommand(() -> shooter.manualHood(-5)));

        shootTrigger.whileTrue(shooter.new ChangeVelocity(RPM.of(6000)));

        hubShootTrigger.whileTrue(new HubShoot(shooter, intake, hopper));
        towerShootTrigger.whileTrue(new TowerShoot(shooter, intake, hopper));
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return configurableAuto.getRoutine().cmd();
    }
}