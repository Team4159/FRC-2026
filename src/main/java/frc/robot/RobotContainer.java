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
import frc.lib.AllianceUtil;
import frc.lib.HIDRumble;
import frc.lib.HIDRumble.RumbleRequest;
import frc.lib.PoseUtil;
import frc.lib.Telemetry;
import frc.robot.commands.AutoLob;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.HubShoot;
import frc.robot.commands.TowerShoot;
import frc.robot.operator.OperatorConstants;
import frc.robot.operator.OperatorConstants.DriveFlag;
import frc.robot.operator.OperatorConstants.DriveMode;
import frc.robot.operator.SingleXboxOperatorModality;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.shooter.FeederConstants.FeederState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.PhotonVision;
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
        configurableAuto = new ConfigurableAuto(autoFactory, drivetrain, shooter, intake, hopper);
        drivetrain.setAutonomousAutoShootCommand(
            new AutoShoot(drivetrain, shooter, hopper, intake, true, Optional.empty())
        );

        // drivetrain bindings
        drivetrain.registerTelemetry(telemetry::telemetrizeDrivetrain);
        RobotModeTriggers.disabled().whileTrue(drivetrain.getDriveCommand(DriveMode.IDLE).ignoringDisable(true));

        // call the function that configures the robot bindings
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.getDriveCommand(DriveMode.TELEOP));

        operatorModality.zero().onTrue(
            Commands.runOnce(() -> {
                HIDRumble.rumble(operatorModality.getHID(), new RumbleRequest(RumbleType.kLeftRumble, 0.5, 0.25));
                drivetrain.seedFieldCentric();
            })
        );

        // teleop mode
        operatorModality
            .slowMode()
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(drivetrain.new DriveFlagToggler(DriveFlag.SLOW_MODE));
        operatorModality
            .driverAssist()
            .and(DriverStation::isTeleopEnabled)
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
            .and(DriverStation::isTeleopEnabled)
            .and(() -> PoseUtil.isPoseBehindAllianceTrenches(AllianceUtil.getAlliance(), drivetrain.getState().Pose))
            .whileTrue(
                new AutoShoot(drivetrain, shooter, hopper, intake, false, Optional.of(operatorModality.getHID()))
            );
        operatorModality
            .hubShoot()
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(new HubShoot(shooter, intake, hopper));
        operatorModality
            .towerShoot()
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(new TowerShoot(shooter, intake, hopper));
        operatorModality
            .autoLob()
            .and(DriverStation::isTeleopEnabled)
            .and(() -> !PoseUtil.isPoseBehindAllianceTrenches(AllianceUtil.getAlliance(), drivetrain.getState().Pose))
            .whileTrue(new AutoLob(drivetrain, shooter, hopper, intake, false));

        operatorModality
            .intake()
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(
                new ParallelCommandGroup(
                    intake.new ChangeStates(IntakeState.DOWN_ON),
                    hopper.new ChangeState(HopperState.FEED)
                )
            ); // .onFalse(intake.new
        // ChangeStates(IntakeState.BOUNCE_UP));
        operatorModality
            .outtake()
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(
                new ParallelCommandGroup(
                    intake.new ChangeStates(IntakeState.DOWN_REVERSE),
                    hopper.new ChangeState(HopperState.REVERSE),
                    shooter.new ChangeFeederState(FeederState.UNJAM)
                )
            );
        operatorModality
            .retractIntake()
            .and(DriverStation::isTeleopEnabled)
            .onTrue(
                new ParallelCommandGroup(
                    intake.new ChangeStates(IntakeState.UP_OFF),
                    hopper.new ChangeState(HopperState.STOP),
                    shooter.new ChangeFeederState(FeederState.STOP)
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
