// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.lib.AllianceUtil;
import frc.lib.PoseUtil;
import frc.lib.Telemetry;
import frc.robot.auto.ConfigurableAuto;
import frc.robot.commands.shoot.AutoLob;
import frc.robot.commands.shoot.AutoShoot;
import frc.robot.commands.shoot.HubShoot;
import frc.robot.commands.shoot.TowerShoot;
import frc.robot.operator.OperatorConstants;
import frc.robot.operator.OperatorConstants.DriveFlag;
import frc.robot.operator.OperatorConstants.DriveMode;
import frc.robot.operator.RumbleFeedback;
import frc.robot.operator.SingleXboxOperatorModality;
import frc.robot.subsystems.drivetrain.DriveFlagToggler;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperSetpoint;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSetpoint;
import frc.robot.subsystems.shooter.FeederConstants.FeederSetpoint;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.PhotonVision;

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
    private final AutoFactory autoFactory = drivetrain.createAutoFactory();
    private final ConfigurableAuto configurableAuto = new ConfigurableAuto(
        autoFactory,
        drivetrain,
        shooter,
        intake,
        hopper
    );

    public RobotContainer() {
        // Choreo Auto
        CommandScheduler.getInstance().schedule(autoFactory.warmupCmd()); // warmup command so auto starts instantly

        // drivetrain
        drivetrain.registerTelemetry(telemetry::telemetrizeDrivetrain);
        RobotModeTriggers.disabled().whileTrue(drivetrain.createDriveCommand(DriveMode.IDLE).ignoringDisable(true));
        drivetrain.setDefaultCommand(drivetrain.createDriveCommand(DriveMode.TELEOP));

        // call the function that configures the robot bindings
        configureBindings();
    }

    private void configureBindings() {
        operatorModality.zero().onTrue(
            Commands.runOnce(() -> {
                RumbleFeedback.zero(operatorModality.getHID());
                drivetrain.seedFieldCentric();
            })
        );

        // teleop mode
        operatorModality
            .slowMode()
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(new DriveFlagToggler(drivetrain, DriveFlag.SLOW_MODE));
        operatorModality
            .driverAssist()
            .and(DriverStation::isTeleopEnabled)
            .onTrue(
                Commands.runOnce(() -> {
                    RumbleFeedback.toggleDriveAssist(operatorModality.getHID());
                    drivetrain
                        .getDriveFlags()
                        .setValue(DriveFlag.DRIVE_ASSIST, !drivetrain.getDriveFlags().getValue(DriveFlag.DRIVE_ASSIST));
                })
            );
        operatorModality
            .autoShoot()
            .and(DriverStation::isTeleopEnabled)
            .and(() -> PoseUtil.isPoseBehindAllianceTrenches(AllianceUtil.getAlliance(), drivetrain.getState().Pose))
            .whileTrue(new AutoShoot(drivetrain, shooter, hopper, intake, true));
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
            .whileTrue(new AutoLob(drivetrain, shooter, hopper, intake, true));
        operatorModality
            .intake()
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(
                Commands.parallel(
                    intake.new ChangeSetpoints(IntakeSetpoint.DOWN_ON),
                    hopper.new ChangeSetpoint(HopperSetpoint.FEED)
                )
            );
        operatorModality
            .outtake()
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(
                Commands.parallel(
                    intake.new ChangeSetpoints(IntakeSetpoint.DOWN_REVERSE),
                    hopper.new ChangeSetpoint(HopperSetpoint.REVERSE),
                    shooter.new ChangeFeederSetpoint(FeederSetpoint.UNJAM)
                )
            );
        operatorModality
            .retractIntake()
            .and(DriverStation::isTeleopEnabled)
            .onTrue(
                Commands.parallel(
                    intake.new ChangeSetpoints(IntakeSetpoint.UP_OFF),
                    hopper.new ChangeSetpoint(HopperSetpoint.STOP),
                    shooter.new ChangeFeederSetpoint(FeederSetpoint.STOP)
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
