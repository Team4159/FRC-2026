// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.HopperConstants.HopperState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.commands.AutoAim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //Controllers
    private final CommandXboxController primaryController = new CommandXboxController(0);
    private final CommandXboxController secondary = new CommandXboxController(1);

    //Primary Triggers
    private final Trigger AutoAimTrigger = primaryController.rightBumper();

    //Secondary Triggers
    private final Trigger feedHopper = secondary.x();
    private final Trigger reverseFeederHopper = secondary.y();

    private final Trigger intakeTrigger = secondary.leftBumper();
    private final Trigger outtakeTrigger = secondary.leftTrigger(0.1);
    private final Trigger stowTrigger = secondary.b();

    //Subsystems
    private final Shooter shooter = new Shooter();
    private final Hopper hopper = new Hopper();
    private final Intake intake = new Intake();
    private final LEDs leds = new LEDs();
    private final Drivetrain drivetrain = new Drivetrain(primaryController);
    private final PhotonVision photonvision = new PhotonVision(drivetrain);

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        //set auto command for drivetrain
        drivetrain.setAutonomousAutoAimCommand(new AutoAim(drivetrain, shooter, hopper, leds, true));
        //Choreo Auto
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, drivetrain);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        autoChooser.addRoutine("Left", autoRoutines::leftAuto);
        autoChooser.addRoutine("Right", autoRoutines::rightAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(drivetrain.getInputX() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(drivetrain.getInputY() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(drivetrain.getInputRotation() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        AutoAimTrigger.whileTrue(new AutoAim(drivetrain, shooter, hopper, leds, false));
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        primaryController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        primaryController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-primaryController.getLeftY(), -primaryController.getLeftX()))
        ));

        primaryController.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        primaryController.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        //TODO: run sysid when robot is built
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // primaryController.back().and(primaryController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // primaryController.back().and(primaryController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // primaryController.start().and(primaryController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // primaryController.start().and(primaryController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        primaryController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        feedHopper.onTrue(new ParallelCommandGroup(shooter.new ChangeFeederState(FeederState.FEED), hopper.new ChangeHopperState(HopperState.FEED)));
        reverseFeederHopper.onTrue(new ParallelCommandGroup(shooter.new ChangeFeederState(FeederState.UNSTUCKFEEDER), hopper.new ChangeHopperState(HopperState.REVERSE)));

        intakeTrigger.whileTrue(intake.new ChangeIntakeState(IntakeState.DOWN_ON));
        outtakeTrigger.whileTrue(intake.new ChangeIntakeState(IntakeState.DOWN_OFF));
        stowTrigger.whileTrue(intake.new ChangeIntakeState(IntakeState.UP_OFF));
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}