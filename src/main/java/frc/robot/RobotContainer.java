// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.ConfigurableAuto;
import frc.lib.HIDRumble;
import frc.lib.HIDRumble.RumbleRequest;
import frc.lib.Telemetry;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.HopperConstants.HopperState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.DriveFlag;
import frc.robot.Constants.OperatorConstants.DriveMode;
import frc.robot.commands.AutoLob;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.HubShoot;
import frc.robot.commands.TowerShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import java.util.Optional;

public class RobotContainer {

    private final CommandXboxController primaryController = new CommandXboxController(
        OperatorConstants.PRIMARY_CONTROLLER_PORT
    );
    // private final CommandXboxController secondaryController = new CommandXboxController(
    //     OperatorConstants.kSecondaryControllerPort
    // );

    private final Trigger zeroTrigger = primaryController.back();
    private final Trigger slowModeTrigger = primaryController.leftBumper();
    private final Trigger driverAssistToggleTrigger = primaryController.y();

    private final Trigger intakeTrigger = primaryController.leftTrigger(0.1);
    private final Trigger outtakeTrigger = primaryController.x();

    private final Trigger autoShootTriggerPrototype = primaryController.rightTrigger(0.1); // do not use prototype
    private final Trigger hubShootTriggerPrototype = primaryController.rightBumper(); // do not use prototype
    private final Trigger autoShootTrigger, hubShootTrigger;
    private final Trigger towerShootTrigger = autoShootTriggerPrototype.and(hubShootTriggerPrototype);
    private final Trigger autoLobTrigger = primaryController.a();

    {
        autoShootTrigger = autoShootTriggerPrototype.and(towerShootTrigger.negate());
        hubShootTrigger = hubShootTriggerPrototype.and(towerShootTrigger.negate());
    }

    // Subsystems
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Hopper hopper = new Hopper();
    private final LEDs leds = new LEDs();
    private final Drivetrain drivetrain = new Drivetrain(primaryController);

    @SuppressWarnings("unused")
    // periodic function inside photon vision class used to send vision data
    private final PhotonVision photonVision = new PhotonVision(drivetrain);

    /* Path follower */
    private final AutoFactory autoFactory;
    private final ConfigurableAuto configurableAuto;

    public RobotContainer() {
        // Choreo Auto
        autoFactory = drivetrain.createAutoFactory();
        CommandScheduler.getInstance().schedule(autoFactory.warmupCmd()); // warmup command so auto starts instantly
        configurableAuto = new ConfigurableAuto(autoFactory, drivetrain, shooter, intake, hopper, leds);
        drivetrain.setAutonomousAutoShootCommand(
            new AutoShoot(drivetrain, shooter, hopper, intake, leds, true, Optional.empty())
        );

        // rumble on collision
        drivetrain.crashTrigger.onTrue(
            Commands.runOnce(() ->
                HIDRumble.rumble(primaryController.getHID(), new RumbleRequest(RumbleType.kRightRumble, 1, 0.5, 1))
            )
        );

        // drivetrain telemetry
        drivetrain.registerTelemetry(Telemetry::telemetrizeDrivetrain);

        // call the function that configures the robot bindings
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(drivetrain.new Drive(DriveMode.TELEOP));
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(drivetrain.new Drive(DriveMode.IDLE).ignoringDisable(true));

        // Reset the field-centric heading on left bumper press.
        zeroTrigger.onTrue(
            Commands.runOnce(() -> {
                HIDRumble.rumble(primaryController.getHID(), new RumbleRequest(RumbleType.kLeftRumble, 0.5, 0.25));
                drivetrain.seedFieldCentric();
            })
        );

        // test mode
        primaryController.y().and(DriverStation::isTest).whileTrue(drivetrain.new Drive(DriveMode.BRAKE));
        primaryController.b().and(DriverStation::isTest).whileTrue(drivetrain.new Drive(DriveMode.POINT));

        // teleop mode
        slowModeTrigger.and(DriverStation::isTeleop).whileTrue(drivetrain.new DriveFlagToggler(DriveFlag.SLOW_MODE));
        driverAssistToggleTrigger.and(DriverStation::isTeleop).onTrue(
            Commands.runOnce(() -> {
                HIDRumble.rumble(primaryController.getHID(), new RumbleRequest(RumbleType.kLeftRumble, 0.5, 0.25));
                drivetrain.setDriveFlagValue(
                    DriveFlag.DRIVE_ASSIST,
                    !drivetrain.getDriveFlagValue(DriveFlag.DRIVE_ASSIST)
                );
            })
        );
        autoShootTrigger
            .and(DriverStation::isTeleop)
            .whileTrue(new AutoShoot(drivetrain, shooter, hopper, intake, leds, false, Optional.of(primaryController)));
        hubShootTrigger.and(DriverStation::isTeleop).whileTrue(new HubShoot(shooter, intake, hopper));
        towerShootTrigger.and(DriverStation::isTeleop).whileTrue(new TowerShoot(shooter, intake, hopper));
        autoLobTrigger
            .and(DriverStation::isTeleop)
            .whileTrue(new AutoLob(drivetrain, shooter, hopper, intake, leds, false));

        intakeTrigger.whileTrue(
            new ParallelCommandGroup(
                intake.new ChangeStates(IntakeState.DOWN_ON),
                hopper.new ChangeState(HopperState.FEED)
            )
        ); // .onFalse(intake.new
        // ChangeStates(IntakeState.BOUNCE_UP));
        outtakeTrigger.whileTrue(
            new ParallelCommandGroup(
                intake.new ChangeStates(IntakeState.DOWN_REV),
                hopper.new ChangeState(HopperState.REVERSE),
                shooter.new ChangeState(FeederState.UNJAM)
            )
        );
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser
         * cmd() is used to get the Choreo AutoRoutine object as a WPILIB Command object
         */
        return configurableAuto.getRoutine().cmd();
    }
}
