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
    private double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double maxAngularSpeed = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                       // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularSpeed * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

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
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(maxSpeed);

    private final CommandXboxController primaryController = new CommandXboxController(0);
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
                drivetrain.applyRequest(() -> drive.withVelocityX(drivetrain.getInputX() * maxSpeed) // Drive forward
                                                                                                     // with negative Y
                                                                                                     // (forward)
                        .withVelocityY(drivetrain.getInputY() * maxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(drivetrain.getInputRotation() * maxAngularSpeed) // Drive counterclockwise
                                                                                             // with negative X (left)
                ));

        AutoAimTrigger.whileTrue(new AutoAim(drivetrain));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        primaryController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        primaryController.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-primaryController.getLeftY(), -primaryController.getLeftX()))));

        primaryController.povUp()
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        primaryController.povDown()
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        primaryController.back().and(primaryController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        primaryController.back().and(primaryController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        primaryController.start().and(primaryController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        primaryController.start().and(primaryController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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
                        MetersPerSecond.of(maxSpeed));
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