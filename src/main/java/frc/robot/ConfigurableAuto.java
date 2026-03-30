package frc.robot;

import java.util.ArrayList;
import java.util.Optional;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.Elastic;
import frc.lib.InstantCommandRunWhenDisabled;
import frc.lib.PoseTrajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoRecovery;
import frc.robot.commands.AutoRecovery.AutoRecoveryMode;
import frc.robot.commands.AutoRecovery.AutoRecoverySide;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;

public class ConfigurableAuto {
    private SendableChooser<String> sideChooser, intakeChooser1, shootChooser1, intakeChooser2, shootChooser2,
            climbSideChooser;

    private Field2d generatedRoutineDisplay = new Field2d();

    private final AutoFactory factory;
    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Intake intake;
    private final Hopper hopper;
    private final LEDs leds;

    private AutoRoutine generatedRoutine;

    public ConfigurableAuto(AutoFactory factory, Drivetrain drivetrain, Shooter shooter, Intake intake, Hopper hopper,
            LEDs leds) {
        // auto factory
        this.factory = factory;

        // subsystems
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;
        this.leds = leds;

        // sendable choosers
        sideChooser = new SendableChooser<>();
        intakeChooser1 = new SendableChooser<>();
        shootChooser1 = new SendableChooser<>();
        intakeChooser2 = new SendableChooser<>();
        shootChooser2 = new SendableChooser<>();
        climbSideChooser = new SendableChooser<>();

        displayChoosers();
    }

    /**
     * displays sendable chooser options for configuration and the generate button
     */
    public void displayChoosers() {
        // side chooser
        sideChooser.addOption("Left", "L");
        sideChooser.addOption("Right", "R");
        sideChooser.addOption("Mid", "M");
        sideChooser.addOption("Mid Left", "ML");
        sideChooser.addOption("Mid Right", "MR");
        sideChooser.setDefaultOption("None", "None");

        // intake chooser 1
        intakeChooser1.addOption("Line", "LineIntake");
        intakeChooser1.addOption("Far", "FarIntake");
        intakeChooser1.addOption("Mid", "MidIntake");
        intakeChooser1.addOption("Close", "CloseIntake");
        intakeChooser1.addOption("Outpost (for middle auto)", "OutpostIntake");
        intakeChooser1.setDefaultOption("None", "None");

        // shoot chooser 1
        shootChooser1.addOption("Shoot", "Shoot");
        shootChooser1.addOption("Shoot and Climb", "Climb");
        shootChooser1.setDefaultOption("None", "None");

        // intake chooser 2
        intakeChooser2.addOption("Line", "LineIntake");
        intakeChooser2.addOption("Far", "FarIntake");
        intakeChooser2.addOption("Mid", "MidIntake");
        intakeChooser2.addOption("Close", "CloseIntake");
        intakeChooser2.setDefaultOption("None", "None");

        // shoot chooser 2
        shootChooser2.addOption("Shoot", "Shoot");
        shootChooser2.addOption("Shoot and Climb", "Climb");
        shootChooser2.setDefaultOption("None", "None");

        climbSideChooser.setDefaultOption("Left", "L");
        climbSideChooser.addOption("Right", "R");

        // display on smartdashboard -> elastic
        SmartDashboard.putData("side", sideChooser);
        SmartDashboard.putData("Intake 1", intakeChooser1);
        SmartDashboard.putData("Shoot 1", shootChooser1);
        SmartDashboard.putData("Intake 2", intakeChooser2);
        SmartDashboard.putData("Shoot 2", shootChooser2);
        SmartDashboard.putData("climb side", climbSideChooser);
        SmartDashboard.putData("generate", new InstantCommandRunWhenDisabled(() -> generateRoutine(true)));
    }

    private AutoRoutine generateRoutine(boolean display) {
        final AutoRoutine routine = factory.newRoutine("Generated Auto");

        // if there direction is none return the default routine (does absolutely
        // nothing)
        final String direction = sideChooser.getSelected();
        if (direction.equals("None")) {
            Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.INFO,
                    "Empty auto generated", "this auto will do absolutely nothing");
            Elastic.sendNotification(notification);
            return routine;
        }

        final String intake1 = intakeChooser1.getSelected();
        final String shoot1 = shootChooser1.getSelected();
        final String intake2 = intakeChooser2.getSelected();
        final String shoot2 = shootChooser2.getSelected();
        final String climbSide = climbSideChooser.getSelected();

        final AutoRecoverySide autoRecoverySide;
        if (direction.equals("L")) {
            autoRecoverySide = AutoRecoverySide.LEFT;
        } else if (direction.equals("R")) {
            autoRecoverySide = AutoRecoverySide.RIGHT;
        } else {
            autoRecoverySide = AutoRecoverySide.MIDDLE;
        }

        if (direction.contains("M")) {
            // outpost auto
            if (intake1.contains("Outpost")) {
                final String startToIntakeName = direction + "StartToMROutpostIntake";
                final String intakeToShootName = "MROutpostIntakeToMRShoot";

                final AutoTrajectory startToIntakeTraj = routine.trajectory(startToIntakeName);
                final AutoTrajectory intakeToShootTraj = routine.trajectory(intakeToShootName);

                routine.active().onTrue(
                        startToIntakeTraj.resetOdometry()
                                .andThen(startToIntakeTraj.cmd())
                                .andThen(intakeToShootTraj.cmd())
                                .andThen(new ParallelDeadlineGroup(
                                        // new WaitCommand(AutoConstants.ShootTime),
                                        new AutoAim(drivetrain, shooter, hopper, intake, leds, false,
                                                Optional.empty()))));

                startToIntakeTraj.atTime("intake").onTrue(intake.new ChangeStates(IntakeState.DOWN_ON));
                startToIntakeTraj.atTime("stopIntake").onTrue(intake.new ChangeStates(IntakeState.DOWN_OFF));

                if (display) {
                    updateField(startToIntakeTraj, intakeToShootTraj);
                }

                generatedRoutine = routine;

                displayGenerationStatus(startToIntakeTraj, intakeToShootTraj);

                return routine;
            }

            final String startToShootName = direction + "StartToShoot";
            final String shootToClimbName = direction + "ShootTo" + climbSide + "Climb";

            final AutoTrajectory startToShootTraj = routine.trajectory(startToShootName);
            final AutoTrajectory shootToClimbTraj = routine.trajectory(shootToClimbName);

            routine.active().onTrue(
                    startToShootTraj.resetOdometry()
                            .andThen(startToShootTraj.cmd())
                            .andThen(new ParallelDeadlineGroup(
                                    // new WaitCommand(AutoConstants.ShootTime),
                                    new AutoAim(drivetrain, shooter, hopper, intake, leds, false, Optional.empty())))
            // .andThen(shootToClimbTraj.cmd())
            // .andThen(new AutoAlign(drivetrain, towerAlignGoal,
            // primaryRobotRelativeTrigger))
            );

            if (display) {
                updateField(startToShootTraj, shootToClimbTraj);
            }

            generatedRoutine = routine;

            displayGenerationStatus(startToShootTraj, shootToClimbTraj);

            return routine;
        }

        final String startToIntake1Name = direction + "StartTo" + direction + intake1;
        final String intake1ToShoot1Name = direction + intake1 + "To" + direction + "Shoot";
        final String shoot1ToIntake2Name = direction + "Shoot" + "To" + direction + intake2;
        final String intake2ToShoot2Name = direction + intake2 + "To" + direction + "Shoot";

        final AutoTrajectory startToIntake1Traj = routine.trajectory(startToIntake1Name);
        final AutoTrajectory intake1ToShoot1Traj = routine.trajectory(intake1ToShoot1Name);
        final AutoTrajectory shoot1ToIntake2Traj = routine.trajectory(shoot1ToIntake2Name);
        final AutoTrajectory intake2ToShoot2Traj = routine.trajectory(intake2ToShoot2Name);

        // if shoot1 is climb, disregard shoot1tointake2 and intake2toshoot2
        if (shoot1.contains("Climb")) {
            final AutoTrajectory shoot1ToClimbTraj = routine.trajectory(direction + "ShootTo" + climbSide + "Climb");
            routine.active().onTrue(
                    startToIntake1Traj.resetOdometry()
                            .andThen(startToIntake1Traj.cmd())
                            .andThen(intake1ToShoot1Traj.cmd())
                            .andThen(new ParallelDeadlineGroup(
                                    new WaitCommand(AutoConstants.ShootTime),
                                    new AutoAim(drivetrain, shooter, hopper, intake, leds, false, Optional.empty())))
                            .andThen(shoot1ToClimbTraj.cmd()));

            displayGenerationStatus(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToClimbTraj);

            if (display) {
                updateField(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToClimbTraj);
            }
        }

        else if (shoot2.contains("Climb")) {
            final AutoTrajectory shoot2ToClimbTraj = routine.trajectory(direction + "ShootTo" + climbSide + "Climb");
            routine.active().onTrue(
                    startToIntake1Traj.resetOdometry()
                            .andThen(startToIntake1Traj.cmd())
                            .andThen(intake1ToShoot1Traj.cmd())
                            .andThen(new AutoRecovery(drivetrain, intake, AutoRecoveryMode.SWEEP, autoRecoverySide,
                                    intake1ToShoot1Traj.getFinalPose().get().getTranslation()))
                            .andThen(new ParallelDeadlineGroup(
                                    new WaitCommand(AutoConstants.ShootTime),
                                    new AutoAim(drivetrain, shooter, hopper, intake, leds, false, Optional.empty())))
                            .andThen(shoot1ToIntake2Traj.cmd())
                            .andThen(intake2ToShoot2Traj.cmd())
                            .andThen(new AutoRecovery(drivetrain, intake, AutoRecoveryMode.SWEEP, autoRecoverySide,
                                    intake2ToShoot2Traj.getFinalPose().get().getTranslation()))
                            .andThen(new ParallelDeadlineGroup(
                                    new WaitCommand(AutoConstants.ShootTime),
                                    new AutoAim(drivetrain, shooter, hopper, intake, leds, false, Optional.empty())))
                            .andThen(shoot2ToClimbTraj.cmd()));

            displayGenerationStatus(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToIntake2Traj, intake2ToShoot2Traj,
                    shoot2ToClimbTraj);

            if (display) {
                updateField(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToIntake2Traj, intake2ToShoot2Traj,
                        shoot2ToClimbTraj);
            }
        } else {
            routine.active().onTrue(
                    startToIntake1Traj.resetOdometry()
                            .andThen(startToIntake1Traj.cmd())
                            .andThen(intake1ToShoot1Traj.cmd())
                            .andThen(new AutoRecovery(drivetrain, intake, AutoRecoveryMode.SWEEP, autoRecoverySide,
                                    intake1ToShoot1Traj.getFinalPose().get().getTranslation()))
                            .andThen(new ParallelDeadlineGroup(
                                    new WaitCommand(AutoConstants.ShootTime),
                                    new AutoAim(drivetrain, shooter, hopper, intake, leds, false, Optional.empty())))
                            .andThen(shoot1ToIntake2Traj.cmd())
                            .andThen(intake2ToShoot2Traj.cmd())
                            .andThen(new AutoRecovery(drivetrain, intake, AutoRecoveryMode.SWEEP, autoRecoverySide,
                                    intake2ToShoot2Traj.getFinalPose().get().getTranslation()))
                            .andThen(new ParallelDeadlineGroup(
                                    new WaitCommand(AutoConstants.ShootTime),
                                    new AutoAim(drivetrain, shooter, hopper, intake, leds, false, Optional.empty()))));

            displayGenerationStatus(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToIntake2Traj, intake2ToShoot2Traj);

            if (display) {
                updateField(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToIntake2Traj, intake2ToShoot2Traj);
            }
        }

        shoot1ToIntake2Traj.atTime("intake").onTrue(intake.new ChangeStates(IntakeState.DOWN_ON));
        shoot1ToIntake2Traj.atTime("stopIntake").onTrue(intake.new ChangeStates(IntakeState.DOWN_OFF));

        startToIntake1Traj.atTime("intake").onTrue(intake.new ChangeStates(IntakeState.DOWN_ON));
        startToIntake1Traj.atTime("stopIntake").onTrue(intake.new ChangeStates(IntakeState.DOWN_OFF));

        // store the routine so don't need to generate at the start of auto
        generatedRoutine = routine;

        return routine;
    }

    /**
     * returns the generated routine if it exists otherwise it generates the routine
     * and returns it
     */
    public AutoRoutine getRoutine() {
        if (generatedRoutine == null)
            return generateRoutine(true);
        return generatedRoutine;
    }

    /**
     * throws an elastic error message if 1 or more of the paths dont exist
     * 
     * @return true if there is at least 1 missing path, false if all paths exist
     */
    public boolean checkForErrors(AutoTrajectory... trajectories) {
        boolean errors = false;
        for (AutoTrajectory trajectory : trajectories) {
            if (trajectory.getRawTrajectory().getPoses().length == 0) {
                String invalidTrajectoryName = trajectory.getRawTrajectory().name();
                Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.ERROR,
                        "Auto Path Generation Failed", invalidTrajectoryName + " is invalid with current settings");
                Elastic.sendNotification(notification);
            }
        }
        return errors;
    }

    public void displayGenerationStatus(AutoTrajectory... trajectories) {
        if (checkForErrors(trajectories)) {
            Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.INFO,
                    "Auto Path Generated With Errors", "this just means some paths are missing/invalid");
            Elastic.sendNotification(notification);
        } else {
            Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.INFO,
                    "Auto Path Generated", "");
            Elastic.sendNotification(notification);
        }
    }

    /**
     * displays an autoroutine on smartdashboard
     * 
     * @param trajectories the Choreo AutoTrajectories that make up the routine
     *                     desired to be displayed
     */
    public void updateField(AutoTrajectory... trajectories) {
        edu.wpi.first.math.trajectory.Trajectory trajectory = new edu.wpi.first.math.trajectory.Trajectory();
        for (int i = 0; i < trajectories.length; i++) {
            Trajectory<SwerveSample> choreoTrajectory = trajectories[i].getRawTrajectory();

            ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
            for (int j = 0; j < choreoTrajectory.getPoses().length; j++) {
                poses.add(choreoTrajectory.getPoses()[j]);
            }
            PoseTrajectory pt = new PoseTrajectory(poses);
            trajectory = trajectory.concatenate(pt);
        }
        generatedRoutineDisplay.getObject("traj").setTrajectory(trajectory);
        SmartDashboard.putData("Generated Routine Display", generatedRoutineDisplay);
    }

}
