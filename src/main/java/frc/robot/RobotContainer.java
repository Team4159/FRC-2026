// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.BirdAuto;
import frc.lib.BirdAuto.AlignmentResult;
import frc.lib.BirdAuto.FieldGoal;
import frc.robot.commands.AutoAim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric cruise = new SwerveRequest.FieldCentric()
            .withDeadband(0).withRotationalDeadband(0)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentricFacingAngle align = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(0).withRotationalDeadband(0)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withHeadingPID(15, 0, 0)
            .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController primaryController = new CommandXboxController(0);
    private final Trigger robotCentricDriveTrigger = primaryController.leftStick();
    private final Trigger AutoAimTrigger = primaryController.rightBumper();

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
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
                    double x = drivetrain.getInputX() * MaxSpeed;
                    double y = drivetrain.getInputY() * MaxSpeed;
                    double rotation = drivetrain.getInputRotation() * MaxAngularRate;
                    if (robotCentricDriveTrigger.getAsBoolean()) {
                        return robotCentricDrive.withVelocityX(x)
                                .withVelocityY(y)
                                .withRotationalRate(rotation);
                    } else {
                        return fieldCentricDrive.withVelocityX(x)
                                .withVelocityY(y)
                                .withRotationalRate(rotation);
                    }
                }));

        AutoAimTrigger.whileTrue(new AutoAim(drivetrain));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        primaryController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        primaryController.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-primaryController.getLeftY(), -primaryController.getLeftX()))));

        // Reset the field-centric heading on left bumper press.
        primaryController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        // return autoChooser.selectedCommand();
        return new Command() {
            BirdAuto bird;
            APConstraints constraints = new APConstraints().withJerk(2).withAcceleration(5);
            APProfile cruiseProfile = new APProfile(constraints)
                    .withErrorXY(Centimeters.of(15.0))
                    .withErrorTheta(Degrees.of(360))
                    .withBeelineRadius(Centimeters.of(18.0));
            APProfile alignmentProfile = new APProfile(constraints)
                    .withErrorXY(Centimeters.of(3.0))
                    .withErrorTheta(Degrees.of(5))
                    .withBeelineRadius(Centimeters.of(5.0));
            Autopilot cruiseAutopilot = new Autopilot(cruiseProfile);
            Autopilot alignmentAutopilot = new Autopilot(alignmentProfile);
            {
                addRequirements(drivetrain);
            }

            @Override
            public void initialize() {
                bird = new BirdAuto(new FieldGoal[] {
                        FieldGoal.OUTPOST,
                        FieldGoal.TRENCH_RIGHT,
                        FieldGoal.TRENCH_LEFT,
                        FieldGoal.DEPOT_RIGHT,
                        FieldGoal.DEPOT_LEFT,
                        FieldGoal.CLIMB_MIDDLE
                });
            }

            @Override
            public void execute() {
                AlignmentResult result = bird.calculateAlignment(cruiseAutopilot, alignmentAutopilot,
                        drivetrain.getState().Pose, drivetrain.getState().Speeds, DriverStation.getAlliance().orElse(Alliance.Blue),
                        MetersPerSecond.of(MaxSpeed));
                if (result.translationOnly()) {
                    drivetrain.setControl(cruise.withVelocityX(result.velocityX()).withVelocityY(result.velocityY()));
                } else {
                    drivetrain.setControl(align.withVelocityX(result.velocityX()).withVelocityY(result.velocityY())
                        .withTargetDirection(result.rotationHeading()));
                }
            }
        };
    }
}