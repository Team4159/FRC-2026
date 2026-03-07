// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.FluentTrigger;
import frc.lib.HIDRumble;
import frc.lib.HIDRumble.RumbleRequest;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.AlignConstants.TowerAlignGoal;
import frc.robot.Constants.ClimberConstants.ClimberState;
import frc.robot.Constants.OperatorConstants.DriveMode;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.HopperConstants.HopperState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoAlign;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private final Telemetry logger = new Telemetry();

    private final CommandXboxController primaryController = new CommandXboxController(
            OperatorConstants.kPrimaryControllerPort);
    private final CommandXboxController secondaryController = new CommandXboxController(
        OperatorConstants.kSecondaryControllerPort);
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
    // private final Trigger primaryMiddleFrontClimbAlignTrigger =
    // primaryController.povUp();
    // private final Trigger primaryMiddleBackClimbAlignTrigger =
    // primaryController.povDown();
    private final Trigger primaryFacingAngleTrigger = primaryController.rightStick();

    //Secondary Triggers
    private final Trigger feedHopperTrigger = secondaryController.x();
    private final Trigger reverseFeederHopperTrigger = secondaryController.y();

    private final Trigger manualHoodPitchUpTrigger = secondaryController.povUp();
    private final Trigger manualHoodPitchDownTrigger = secondaryController.povDown();

    private final Trigger intakeTrigger = secondaryController.leftBumper();
    private final Trigger outtakeTrigger = secondaryController.leftTrigger(0.1);
    private final Trigger compressIntakeTrigger = secondaryController.a();
    private final Trigger bounceIntakeTrigger = secondaryController.b();

    private final Trigger raiseClimbTrigger = secondaryController.povLeft();
    private final Trigger lowerClimbTrigger = secondaryController.povRight();

    //Subsystems
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Hopper hopper = new Hopper();
    private final LEDs leds = new LEDs();
    private final Climber climber = new Climber();
    private final Drivetrain drivetrain = new Drivetrain(primaryController);

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();
    private final ConfigurableAuto configurableAuto;

    public RobotContainer() {
        //set auto command for drivetrain
        drivetrain.setAutonomousAutoAimCommand(new AutoAim(drivetrain, shooter, hopper, leds, true));
        //Choreo Auto
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, drivetrain);

        configurableAuto = new ConfigurableAuto(autoFactory, drivetrain, shooter, intake, hopper, leds);

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
                primaryRobotRelativeTrigger::getAsBoolean));
        primaryAutoAimTrigger.whileTrue(new AutoAim(drivetrain, shooter, hopper, leds, false));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.new Drive(DriveMode.IDLE).ignoringDisable(true));

        // test mode
        primaryController.a().and(DriverStation::isTest).whileTrue(drivetrain.new Drive(DriveMode.BRAKE));
        primaryController.b().and(DriverStation::isTest).whileTrue(drivetrain.new Drive(DriveMode.POINT));

        // teleop mode
        primaryRobotManualAlignModeTrigger.and(DriverStation::isTeleop)
                .whileTrue(drivetrain.new Drive(DriveMode.MANUAL_ALIGN,
                        primaryRobotRelativeTrigger::getAsBoolean));
        primaryIntakeModeTrigger.and(DriverStation::isTeleop).whileTrue(drivetrain.new Drive(DriveMode.INTAKE));
        primaryRadialModeTrigger.and(DriverStation::isTeleop).whileTrue(drivetrain.new Drive(DriveMode.RADIAL));
        primaryReduceSpeedTrigger.and(DriverStation::isTeleop).onChange(
                Commands.runOnce(() -> drivetrain.enableReduceSpeed(primaryReduceSpeedTrigger.getAsBoolean())));
        primaryDriverAssistTrigger.and(DriverStation::isTeleop).onChange(
                Commands.runOnce(() -> drivetrain.enableDriveAssist(!primaryDriverAssistTrigger.getAsBoolean())));
        primaryLeftClimbAlignTrigger.and(DriverStation::isTeleop)
                .onTrue(new AutoAlign(drivetrain, TowerAlignGoal.LEFT, primaryRobotRelativeTrigger));
        primaryRightClimbAlignTrigger.and(DriverStation::isTeleop)
                .onTrue(new AutoAlign(drivetrain, TowerAlignGoal.RIGHT, primaryRobotRelativeTrigger));
        // primaryMiddleFrontClimbAlignTrigger.and(DriverStation::isTeleop).onTrue(new
        // AutoAlign(drivetrain, TowerAlignGoal.MIDDLE_FRONT,
        // primaryRobotRelativeTrigger));
        // primaryMiddleBackClimbAlignTrigger.and(DriverStation::isTeleop).onTrue(new
        // AutoAlign(drivetrain, TowerAlignGoal.MIDDLE_BACK,
        // primaryRobotRelativeTrigger));
        FluentTrigger.build()
                .setDefault(Commands.runOnce(drivetrain::clearDesiredRotation))
                .bind(primaryFacingAngleTrigger.and(DriverStation::isTeleop),
                        Commands.runOnce(() -> HIDRumble.rumble(primaryController.getHID(),
                                new RumbleRequest(RumbleType.kLeftRumble, 0.25, 0.25)))
                                .andThen(Commands.run(() -> {
                                    Optional<Rotation2d> desiredRotation = drivetrain.getInputRotation();
                                    if (desiredRotation.isEmpty()) {
                                        return;
                                    }
                                    drivetrain.setDesiredRotation(desiredRotation.get());
                                }))
                                .finallyDo(() -> HIDRumble.rumble(primaryController.getHID(),
                                        new RumbleRequest(RumbleType.kLeftRumble, 0.25, 0.25))));

        // Reset the field-centric heading on left bumper press.
        primaryZeroTrigger.onTrue(Commands.runOnce(() -> {
            HIDRumble.rumble(primaryController.getHID(), new RumbleRequest(RumbleType.kLeftRumble, 0.5, 0.25));
            drivetrain.seedFieldCentric();
        }));

        drivetrain.registerTelemetry(logger::telemeterize);

        feedHopperTrigger.onTrue(new ParallelCommandGroup(shooter.new ChangeState(FeederState.FEED), hopper.new ChangeState(HopperState.FEED)));
        reverseFeederHopperTrigger.onTrue(new ParallelCommandGroup(shooter.new ChangeState(FeederState.UNSTUCKFEEDER), hopper.new ChangeState(HopperState.REVERSE)));

        //intake
        intakeTrigger.whileTrue(intake.new ChangeStates(IntakeState.DOWN_ON));
        outtakeTrigger.whileTrue(intake.new ChangeStates(IntakeState.DOWN_REV));
        compressIntakeTrigger.whileTrue(intake.new CompressIntake());
        bounceIntakeTrigger.whileTrue(intake.new BounceIntake());

        raiseClimbTrigger.whileTrue(climber.new ChangeState(ClimberState.CLIMB));
        lowerClimbTrigger.whileTrue(climber.new ChangeState(ClimberState.DOWN));
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return configurableAuto.getRoutine().cmd();
    }
}