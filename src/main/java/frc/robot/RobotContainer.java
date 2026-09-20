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
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
import frc.robot.operator.SingleXboxOperatorModality;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import java.util.Optional;

public class RobotContainer {

    private final Telemetry telemetry = new Telemetry();

    private final SingleXboxOperatorModality operatorModality = new SingleXboxOperatorModality(
        OperatorConstants.PRIMARY_CONTROLLER_PORT,
        OperatorConstants.PRIMARY_TRIGGER_THRESHOLD
    );

    // Subsystems
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Hopper hopper = new Hopper();
    private final LEDs leds = new LEDs();
    private final Drivetrain drivetrain = new Drivetrain(operatorModality);

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

        // drivetrain bindings
        drivetrain.registerTelemetry(telemetry::telemetrizeDrivetrain);
        RobotModeTriggers.disabled().whileTrue(drivetrain.new Drive(DriveMode.IDLE).ignoringDisable(true));

        // call the function that configures the robot bindings
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.new Drive(DriveMode.TELEOP));

        operatorModality.zero().onTrue(
            Commands.runOnce(() -> {
                HIDRumble.rumble(operatorModality.getHID(), new RumbleRequest(RumbleType.kLeftRumble, 0.5, 0.25));
                drivetrain.seedFieldCentric();
            })
        );

        // teleop mode
        operatorModality
            .slowMode()
            .and(DriverStation::isTeleop)
            .whileTrue(drivetrain.new DriveFlagToggler(DriveFlag.SLOW_MODE));
        operatorModality
            .driverAssist()
            .and(DriverStation::isTeleop)
            .onTrue(
                Commands.runOnce(() -> {
                    HIDRumble.rumble(operatorModality.getHID(), new RumbleRequest(RumbleType.kLeftRumble, 0.5, 0.25));
                    drivetrain.setDriveFlagValue(
                        DriveFlag.DRIVE_ASSIST,
                        !drivetrain.getDriveFlagValue(DriveFlag.DRIVE_ASSIST)
                    );
                })
            );
        operatorModality
            .autoShoot()
            .and(DriverStation::isTeleop)
            .whileTrue(
                new AutoShoot(drivetrain, shooter, hopper, intake, leds, false, Optional.of(operatorModality.getHID()))
            );
        operatorModality
            .hubShoot()
            .and(DriverStation::isTeleop)
            .whileTrue(new HubShoot(shooter, intake, hopper));
        operatorModality
            .towerShoot()
            .and(DriverStation::isTeleop)
            .whileTrue(new TowerShoot(shooter, intake, hopper));
        operatorModality
            .autoLob()
            .and(DriverStation::isTeleop)
            .whileTrue(new AutoLob(drivetrain, shooter, hopper, intake, leds, false));

        operatorModality
            .intake()
            .whileTrue(
                new ParallelCommandGroup(
                    intake.new ChangeStates(IntakeState.DOWN_ON),
                    hopper.new ChangeState(HopperState.FEED)
                )
            ); // .onFalse(intake.new
        // ChangeStates(IntakeState.BOUNCE_UP));
        operatorModality
            .outtake()
            .whileTrue(
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

    public void periodic() {
        telemetry.update();
    }
}
