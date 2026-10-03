package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceUtil;
import frc.lib.Elastic;
import frc.lib.PoseTrajectory;
import frc.lib.PoseUtil;
import frc.robot.commands.shoot.AutoShoot;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSetpoint;
import frc.robot.subsystems.shooter.Shooter;
import java.util.ArrayList;
import java.util.Collections;

public class ConfigurableAuto {

    /**configurable auto uses many different sendable choosers to choose the desired endpoints and/or behaviors
    the sideChooser chooses the starting point as well as which side trajectories to use later on (there are different trajectories for left and right)
    the intake choosers are mainly for choosing how far to intake (line, far(but still on own alliance side), mid, and close) as well as an outpost intake mode for mid autos
    shoot choosers were originally relevant for if the robot should climb after shooting, this is no longer the case. now it can be used to select the bump auto mode (which is closer for more accurate shooting) but it was unreliable (not enough testing) and currently only exists for left side far and close intaking

    these choosers are just of type String and they will correspond to the trajectory names for the configurable system to work properly*/
    private final SendableChooser<Time> startDelayChooser = AutoDashboardConfiguration.startDelayChooser();
    private final SendableChooser<Time> shootTimeChooser = AutoDashboardConfiguration.shootTimeChooser();
    private final SendableChooser<String> sideChooser = AutoDashboardConfiguration.sideChooser();
    private final SendableChooser<String> intakeChooser1 = AutoDashboardConfiguration.intakeChooser();
    private final SendableChooser<String> shootChooser1 = AutoDashboardConfiguration.shootChooser();
    private final SendableChooser<String> intakeChooser2 = AutoDashboardConfiguration.intakeChooser();
    private final SendableChooser<String> shootChooser2 = AutoDashboardConfiguration.shootChooser();

    /** this field is on the auto tab of elastic to display the auto path once it is generated
    the term "generated" here is not actually generating the choreo paths themselves, but it does take awhile to load each individual path on roborio which is why it needs to be "generated" before the match starts*/
    private final Field2d generatedRoutineDisplay = new Field2d();

    /** AutoFactory used by choreo to make AutoRoutine objects that uses the swerve functions specified
    basically it is how swerve path following functions are implemented
    check out createAutoFactory() in drivetrain to see how it is used*/
    private final AutoFactory factory;
    //subsystems
    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Intake intake;
    private final Hopper hopper;

    /** the routine that is saved after generation */
    private AutoRoutine generatedRoutine;

    /** @param factory the Choreo AutoFactory object
     * the rest should be self explanatory
     */
    public ConfigurableAuto(AutoFactory factory, Drivetrain drivetrain, Shooter shooter, Intake intake, Hopper hopper) {
        // auto factory
        this.factory = factory;

        // subsystems
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;

        displayWidgets();
    }

    /**
     * returns the generated routine if it exists otherwise it generates the routine
     * and returns it
     */
    public AutoRoutine getRoutine() {
        if (generatedRoutine == null) {
            generatedRoutine = generateRoutine();
        }
        return generatedRoutine;
    }

    /**
     * throws an elastic error message if 1 or more of the paths dont exist
     *
     * @return true if there is at least 1 missing path, false if all paths exist
     */
    private boolean checkForErrors(AutoTrajectory... trajectories) {
        boolean errors = false;
        //loop through all trajectories
        for (AutoTrajectory trajectory : trajectories) {
            //if a trajectory is empty (it could not be loaded from choreo because it doesnt exist)
            if (trajectory.getRawTrajectory().getPoses().length != 0) {
                continue;
            }
            errors = true;
            //get the name of the invalid trajectory
            String invalidTrajectoryName = trajectory.getRawTrajectory().name();
            //send an error message that says the name of the trajectory, this error is likely caused by an invalid combination of trajectories inputted into the sendable choosers
            Elastic.Notification notification = new Elastic.Notification(
                Elastic.NotificationLevel.ERROR,
                "Auto Path Generation Failed",
                invalidTrajectoryName + " is invalid with current settings"
            );
            Elastic.sendNotification(notification);
        }
        return errors;
    }

    /** displays the generation status elastic notification of the given trajectories */
    private void displayGenerationStatus(AutoTrajectory... trajectories) {
        //check all trajectories for errors
        if (checkForErrors(trajectories)) {
            //send an info notification saying the trajectory was generated with errors (checkForErrors() already sends actual error type messages)
            Elastic.Notification notification = new Elastic.Notification(
                Elastic.NotificationLevel.INFO,
                "Auto Path Generated With Errors",
                "this just means some paths are missing/invalid"
            );
            Elastic.sendNotification(notification);
        } else {
            //if all the trajectories are good just say auto path generated
            Elastic.Notification notification = new Elastic.Notification(
                Elastic.NotificationLevel.INFO,
                "Auto Path Generated",
                ""
            );
            Elastic.sendNotification(notification);
        }
    }

    /**
     * displays an autoroutine on smartdashboard
     *
     * @param trajectories the Choreo AutoTrajectories that make up the routine
     *                     desired to be displayed
     */
    private void updateField(AutoTrajectory... autoTrajectories) {
        //make a WPILIB trajectory object (so it can be displayed on a field2d)
        var trajectory = new edu.wpi.first.math.trajectory.Trajectory();
        //loop though all the trajectories (these are not WPILIB trajectories but Choreo AutoTrajectories)
        for (AutoTrajectory autoTrajectory : autoTrajectories) {
            //get the raw trajectories from choreo (which is a Choreo class confusingly also called Trajectory 😭)
            Trajectory<SwerveSample> choreoTrajectory = autoTrajectory.getRawTrajectory();

            //loop through the choreo trajectory to get an ArrayList of Pose2ds
            ArrayList<Pose2d> poses = new ArrayList<>();
            Collections.addAll(poses, choreoTrajectory.getPoses());
            if (AllianceUtil.getAlliance().equals(Alliance.Red)) {
                for (int i = 0; i < poses.size(); i++) {
                    poses.set(i, PoseUtil.flipPoseAlongMiddleXY(poses.get(i)));
                }
            }

            //make a new PoseTrajectory object with the array of Pose2ds
            //a PoseTrajectory is a WPILIB Trajectory with a custom constructor that allows it to be created off of an array of Pose2ds, yeah its janky but it works for the sole purpose of displaying trajectories on the Field2d
            PoseTrajectory pt = new PoseTrajectory(poses);
            //concatenate the PoseTrajectory to the regular WPILIB trajectory(works because PoseTrajectory class is derived from the WPILIB trajectory)
            trajectory = trajectory.concatenate(pt);
        }
        //display the trajectory on the Field2d (generatedRoutineDisplay)
        generatedRoutineDisplay.getObject("traj").setTrajectory(trajectory);
    }

    /** adds options to the choosers
     * default option is always "none"
     * this system will not stop you from inputting an invalid trajectory (such as left -> outpost)
     * and instead it will just give you an error message during generation
     * last year we updated the options of choosers that came after each time a chooser is updated, but due to how elastic works it never changed the display and made the change in options unclear to the drivers
     */
    /**
     * displays the sendable chooser options for configuration and the generate button
     */
    private void displayWidgets() {
        AutoDashboardConfiguration.publish(
            startDelayChooser,
            shootTimeChooser,
            sideChooser,
            intakeChooser1,
            shootChooser1,
            intakeChooser2,
            shootChooser2,
            Commands.runOnce(() -> {
                generatedRoutine = generateRoutine();
            }).ignoringDisable(true),
            generatedRoutineDisplay
        );
    }

    /** @param display should the generated trajectory be added to the generatedRoutineDisplay as a trajectory
     * will send elastic notifications on the status of the auto
     * @return an AutoRoutine object of the generated routine
     */
    private AutoRoutine generateRoutine() {
        final AutoRoutine routine = factory.newRoutine("Generated Auto");

        // if the direction is none return the default routine (does absolutely nothing) and send a special notification to let the drivers know they selected a useless auto (could be good if auto is cooked though)
        if (sideChooser.getSelected().equals("None")) {
            Elastic.Notification notification = new Elastic.Notification(
                Elastic.NotificationLevel.INFO,
                "Empty auto generated",
                "this auto will do absolutely nothing"
            );
            Elastic.sendNotification(notification);
            return routine;
        }

        //if the direction has a capital "M" then it is a mid auto (ML, M, or MR)
        if (sideChooser.getSelected().contains("M")) {
            // check if the 1st intake has "Outpost"
            //if so generate an outpost auto
            // outpost auto
            if (intakeChooser1.getSelected().contains("Outpost")) {
                return generateOutpostRoutine(routine);
            }

            return generateMiddleRoutine(routine);
        }

        return generateStandardRoutine(routine);
    }

    private AutoRoutine generateOutpostRoutine(AutoRoutine routine) {
        // TODO: there are currently no outpost routines
        Time startDelay = startDelayChooser.getSelected();
        String side = sideChooser.getSelected();
        String startToIntakeName = AutoPathNames.outpostStartToIntake(side);
        String intakeToShootName = AutoPathNames.outpostIntakeToShoot();

        //load the trajectories with the names
        final AutoTrajectory startToIntakeTraj = routine.trajectory(startToIntakeName);
        final AutoTrajectory intakeToShootTraj = routine.trajectory(intakeToShootName);

        //routine.active().onTrue() runs at the start of the auto
        routine
            .active()
            .onTrue(
                startToIntakeTraj
                    .resetOdometry()
                    .andThen(Commands.waitTime(startDelay))
                    .andThen(startToIntakeTraj.cmd())
                    .andThen(intakeToShootTraj.cmd())
                    .andThen(getAutoShoot())
            );

        //choreo marker behavior
        //used to tell robot when to intake and stop intaking based on markers in the intake trajectories
        startToIntakeTraj.atTime("intake").onTrue(intake.new ChangeSetpoints(IntakeSetpoint.DOWN_ON));
        startToIntakeTraj.atTime("stopIntake").onTrue(intake.new ChangeSetpoints(IntakeSetpoint.DOWN_OFF));

        updateField(startToIntakeTraj, intakeToShootTraj);
        displayGenerationStatus(startToIntakeTraj, intakeToShootTraj);

        return routine;
    }

    private AutoRoutine generateMiddleRoutine(AutoRoutine routine) {
        Time startDelay = startDelayChooser.getSelected();
        String side = sideChooser.getSelected();
        String startToShootName = AutoPathNames.middleStartToShoot(side);

        final AutoTrajectory startToShootTraj = routine.trajectory(startToShootName);

        routine
            .active()
            .onTrue(
                startToShootTraj
                    .resetOdometry()
                    .andThen(Commands.waitTime(startDelay))
                    .andThen(startToShootTraj.cmd())
                    .andThen(getAutoShoot())
            );

        updateField(startToShootTraj);
        displayGenerationStatus(startToShootTraj);

        return routine;
    }

    private AutoRoutine generateStandardRoutine(AutoRoutine routine) {
        Time startDelay = startDelayChooser.getSelected();
        Time shootTime = shootTimeChooser.getSelected();
        String side = sideChooser.getSelected();
        String intake1 = intakeChooser1.getSelected();
        String shoot1 = shootChooser1.getSelected();
        String intake2 = intakeChooser2.getSelected();
        String shoot2 = shootChooser2.getSelected();

        //create the names of the trajectories from the sendable chooser data concatenated together along with other words like "To" so it matches the names of the choreo trajectories
        String startToIntake1Name = AutoPathNames.startToIntake(intake1);
        String intake1ToShoot1Name = AutoPathNames.intakeToShoot(intake1, shoot1);
        String shoot1ToIntake2Name = AutoPathNames.shootToIntake(shoot1, intake2);
        String intake2ToShoot2Name = AutoPathNames.intakeToShoot(intake2, shoot2);

        //load the AutoTrajectories using the names
        AutoTrajectory startToIntake1Traj = routine.trajectory(startToIntake1Name);
        AutoTrajectory intake1ToShoot1Traj = routine.trajectory(intake1ToShoot1Name);
        AutoTrajectory shoot1ToIntake2Traj = routine.trajectory(shoot1ToIntake2Name);
        AutoTrajectory intake2ToShoot2Traj = routine.trajectory(intake2ToShoot2Name);
        if (side.contains("L")) {
            startToIntake1Traj = startToIntake1Traj.mirrorY();
            intake1ToShoot1Traj = intake1ToShoot1Traj.mirrorY();
            shoot1ToIntake2Traj = shoot1ToIntake2Traj.mirrorY();
            intake2ToShoot2Traj = intake2ToShoot2Traj.mirrorY();
        }

        routine.active().onTrue(
            //resetOdometry() at the start sets the robot inital position to the start point of the 1st trajectory
            startToIntake1Traj
                .resetOdometry()
                .andThen(Commands.waitTime(startDelay))
                .andThen(startToIntake1Traj.cmd())
                .andThen(shooter::revFlywheel)
                .andThen(intake1ToShoot1Traj.cmd())
                .andThen(Commands.deadline(Commands.waitTime(shootTime), getAutoShoot()))
                .andThen(shoot1ToIntake2Traj.cmd())
                .andThen(shooter::revFlywheel)
                .andThen(intake2ToShoot2Traj.cmd())
                .andThen(getAutoShoot())
        );

        //choreo marker behavior
        //used to tell robot when to intake and stop intaking based on markers in the intake trajectories
        startToIntake1Traj.atTime("intake").onTrue(intake.new ChangeSetpoints(IntakeSetpoint.DOWN_ON));
        startToIntake1Traj.atTime("stopIntake").onTrue(intake.new ChangeSetpoints(IntakeSetpoint.DOWN_OFF));

        shoot1ToIntake2Traj.atTime("intake").onTrue(intake.new ChangeSetpoints(IntakeSetpoint.DOWN_ON));
        shoot1ToIntake2Traj.atTime("stopIntake").onTrue(intake.new ChangeSetpoints(IntakeSetpoint.DOWN_OFF));

        updateField(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToIntake2Traj, intake2ToShoot2Traj);
        displayGenerationStatus(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToIntake2Traj, intake2ToShoot2Traj);

        return routine;
    }

    private AutoShoot getAutoShoot() {
        return new AutoShoot(drivetrain, shooter, hopper, intake, false);
    }
}
