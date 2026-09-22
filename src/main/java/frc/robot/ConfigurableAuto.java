package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.AllianceUtil;
import frc.lib.Elastic;
import frc.lib.PoseTrajectory;
import frc.lib.PoseUtil;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.commands.AutoShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class ConfigurableAuto {

    /**configurable auto uses many different sendable choosers to choose the desired endpoints and/or behaviors
    the sideChooser chooses the starting point as well as which side trajectories to use later on (there are different trajectories for left and right)
    the intake choosers are mainly for choosing how far to intake (line, far(but still on own alliance side), mid, and close) as well as an outpost intake mode for mid autos
    shoot choosers were originally relevant for if the robot should climb after shooting, this is no longer the case. now it can be used to select the bump auto mode (which is closer for more accurate shooting) but it was unreliable (not enough testing) and currently only exists for left side far and close intaking

    these choosers are just of type String and they will correspond to the trajectory names for the configurable system to work properly*/
    private SendableChooser<String> sideChooser, intakeChooser1, shootChooser1, intakeChooser2, shootChooser2;
    //climb chooser and everything related to climb has been commented but it was originally used to select which side to climb on
    //climbSideChooser;

    /** this field is on the auto tab of elastic to display the auto path once it is generated
    the term "generated" here is not actually generating the choreo paths themselves, but it does take awhile to load each individual path on roborio which is why it needs to be "generated" before the match starts*/
    private Field2d generatedRoutineDisplay = new Field2d();

    /** AutoFactory used by choreo to make AutoRoutine objects that uses the swerve functions specified
    basically it is how swerve path following functions are implemented
    check out createAutoFactory() in drivetrain to see how it is used*/
    private final AutoFactory factory;
    //subsystems
    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Intake intake;
    private final Hopper hopper;
    private final LEDs leds;

    /** the routine that is saved after generation */
    private AutoRoutine generatedRoutine;

    /** @param factory the Choreo AutoFactory object
     * the rest should be self explanatory
     */
    public ConfigurableAuto(
        AutoFactory factory,
        Drivetrain drivetrain,
        Shooter shooter,
        Intake intake,
        Hopper hopper,
        LEDs leds
    ) {
        // auto factory
        this.factory = factory;

        // subsystems
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;
        this.leds = leds;

        // sendable choosers
        // initialize the sendablechooser objects
        sideChooser = new SendableChooser<>();
        intakeChooser1 = new SendableChooser<>();
        shootChooser1 = new SendableChooser<>();
        intakeChooser2 = new SendableChooser<>();
        shootChooser2 = new SendableChooser<>();
        //climbSideChooser = new SendableChooser<>();

        addChooserOptions();
        displayWidgets();
    }

    /** adds options to the choosers
     * default option is always "none"
     * this system will not stop you from inputting an invalid trajectory (such as left -> outpost)
     * and instead it will just give you an error message during generation
     * last year we updated the options of choosers that came after each time a chooser is updated, but due to how elastic works it never changed the display and made the change in options unclear to the drivers
     */
    private void addChooserOptions() {
        // side chooser
        addSideOptions(sideChooser);

        // intake chooser 1
        addIntakeOptions(intakeChooser1);

        // shoot chooser 1
        addShootOptions(shootChooser1);

        // intake chooser 2
        addIntakeOptions(intakeChooser2);

        // shoot chooser 2
        addShootOptions(shootChooser2);
    }

    /**
     * displays the sendable chooser options for configuration and the generate button
     */
    private void displayWidgets() {
        // display on smartdashboard -> elastic
        SmartDashboard.putData("Auto/Side", sideChooser);
        SmartDashboard.putData("Auto/Intake 1", intakeChooser1);
        SmartDashboard.putData("Auto/Shoot 1", shootChooser1);
        SmartDashboard.putData("Auto/Intake 2", intakeChooser2);
        SmartDashboard.putData("Auto/Shoot 2", shootChooser2);
        //SmartDashboard.putData("Auto/Climb Side", climbSideChooser);
        SmartDashboard.putData("Auto/Generate", Commands.runOnce(() -> generateRoutine(true)).ignoringDisable(true));
        SmartDashboard.putData("Auto/Generated Routine Display", generatedRoutineDisplay);
    }

    /** @param display should the generated trajectory be added to the generatedRoutineDisplay as a trajectory
     * will send elastic notifications on the status of the auto
     * @return an AutoRoutine object of the generated routine
     */
    private AutoRoutine generateRoutine(boolean display) {
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
                return generateOutpostRoutine(routine, display);
            }

            return generateMiddleRoutine(routine, display);
        }

        return generateStandardRoutine(routine, display);
    }

    /**
     * returns the generated routine if it exists otherwise it generates the routine
     * and returns it
     */
    public AutoRoutine getRoutine() {
        if (generatedRoutine == null) {
            return generateRoutine(true);
        }
        return generatedRoutine;
    }

    /**
     * throws an elastic error message if 1 or more of the paths dont exist
     *
     * @return true if there is at least 1 missing path, false if all paths exist
     */
    public boolean checkForErrors(AutoTrajectory... trajectories) {
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
    public void displayGenerationStatus(AutoTrajectory... trajectories) {
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
    public void updateField(AutoTrajectory... autoTrajectories) {
        //make a WPILIB trajectory object (so it can be displayed on a field2d)
        edu.wpi.first.math.trajectory.Trajectory trajectory = new edu.wpi.first.math.trajectory.Trajectory();
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

    private AutoRoutine generateOutpostRoutine(AutoRoutine routine, boolean display) {
        // TODO: there are currently no outpost routines
        final String direction = sideChooser.getSelected();
        //these are the names of the trajectories
        //for the outpost auto the only configurable part is the start point though
        final String startToIntakeName = direction + "StartToMROutpostIntake";
        final String intakeToShootName = "MROutpostIntakeToMRShoot";

        //load the trajectories with the names
        final AutoTrajectory startToIntakeTraj = routine.trajectory(startToIntakeName);
        final AutoTrajectory intakeToShootTraj = routine.trajectory(intakeToShootName);

        //routine.active().onTrue() runs at the start of the auto
        routine.active().onTrue(
            //resetOdometry() at the start sets the robot inital position to the start point of the 1st trajectory
            startToIntakeTraj
                .resetOdometry()
                //start -> outpost intake
                .andThen(startToIntakeTraj.cmd())
                //outpost intake -> shooting position
                .andThen(intakeToShootTraj.cmd())
                .andThen(getAutoShoot())
        );

        //choreo marker behavior
        //used to tell robot when to intake and stop intaking based on markers in the intake trajectories
        startToIntakeTraj.atTime("intake").onTrue(intake.new ChangeStates(IntakeState.DOWN_ON));
        startToIntakeTraj.atTime("stopIntake").onTrue(intake.new ChangeStates(IntakeState.DOWN_OFF));

        //update the display field if the display boolean is true
        if (display) {
            updateField(startToIntakeTraj, intakeToShootTraj);
        }
        generatedRoutine = routine;
        displayGenerationStatus(startToIntakeTraj, intakeToShootTraj);

        return generatedRoutine;
    }

    private AutoRoutine generateMiddleRoutine(AutoRoutine routine, boolean display) {
        final String direction = sideChooser.getSelected();
        final String startToShootName = direction + "StartToShoot";

        final AutoTrajectory startToShootTraj = routine.trajectory(startToShootName);

        routine.active().onTrue(
            //resetOdometry() at the start sets the robot inital position to the start point of the 1st trajectory
            startToShootTraj
                .resetOdometry()
                //start -> shooting postion
                //the regular mid auto just scores the preloads
                .andThen(startToShootTraj.cmd())
                .andThen(getAutoShoot())
        );

        if (display) {
            updateField(startToShootTraj);
        }
        generatedRoutine = routine;
        displayGenerationStatus(startToShootTraj);

        return generatedRoutine;
    }

    private AutoRoutine generateStandardRoutine(AutoRoutine routine, boolean display) {
        final String direction = sideChooser.getSelected();
        //get all the chooser results as strings to make things cleaner
        final String intake1 = intakeChooser1.getSelected();
        final String shoot1 = shootChooser1.getSelected();
        final String intake2 = intakeChooser2.getSelected();
        final String shoot2 = shootChooser2.getSelected();

        //create the names of the trajectories from the sendable chooser data concatenated together along with other words like "To" so it matches the names of the choreo trajectories
        final String startToIntake1Name = "StartTo" + intake1;
        final String intake1ToShoot1Name = intake1 + "To" + shoot1;
        final String shoot1ToIntake2Name = shoot1 + "To" + intake2;
        final String intake2ToShoot2Name = intake2 + "To" + shoot2;

        //load the AutoTrajectories using the names
        AutoTrajectory startToIntake1Traj = routine.trajectory(startToIntake1Name);
        AutoTrajectory intake1ToShoot1Traj = routine.trajectory(intake1ToShoot1Name);
        AutoTrajectory shoot1ToIntake2Traj = routine.trajectory(shoot1ToIntake2Name);
        AutoTrajectory intake2ToShoot2Traj = routine.trajectory(intake2ToShoot2Name);
        if (direction.contains("L")) {
            startToIntake1Traj = startToIntake1Traj.mirrorY();
            intake1ToShoot1Traj = intake1ToShoot1Traj.mirrorY();
            shoot1ToIntake2Traj = shoot1ToIntake2Traj.mirrorY();
            intake2ToShoot2Traj = intake2ToShoot2Traj.mirrorY();
        }

        routine.active().onTrue(
            //resetOdometry() at the start sets the robot inital position to the start point of the 1st trajectory
            startToIntake1Traj
                .resetOdometry()
                //start -> intake 1
                .andThen(startToIntake1Traj.cmd())
                //intake 1 -> shoot 1
                .andThen(shooter::revShooter)
                .andThen(intake1ToShoot1Traj.cmd())
                .andThen(
                    new ParallelDeadlineGroup(
                        new WaitCommand(AutoConstants.SHOOT_TIME),
                        getAutoShoot()
                    )
                )
                //shoot 1 -> intake 2
                .andThen(shoot1ToIntake2Traj.cmd())
                //intake 2 -> shoot 2
                .andThen(shooter::revShooter)
                .andThen(intake2ToShoot2Traj.cmd())
                .andThen(getAutoShoot())
        );

        //choreo marker behavior
        //used to tell robot when to intake and stop intaking based on markers in the intake trajectories
        startToIntake1Traj.atTime("intake").onTrue(intake.new ChangeStates(IntakeState.DOWN_ON));
        startToIntake1Traj.atTime("stopIntake").onTrue(intake.new ChangeStates(IntakeState.DOWN_OFF));

        shoot1ToIntake2Traj.atTime("intake").onTrue(intake.new ChangeStates(IntakeState.DOWN_ON));
        shoot1ToIntake2Traj.atTime("stopIntake").onTrue(intake.new ChangeStates(IntakeState.DOWN_OFF));

        if (display) {
            updateField(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToIntake2Traj, intake2ToShoot2Traj);
        }
        generatedRoutine = routine;
        displayGenerationStatus(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToIntake2Traj, intake2ToShoot2Traj);

        return generatedRoutine;
    }

    private AutoShoot getAutoShoot() {
        //auto aim(autonomous mode is false because the point of autonomous mode is for SOTM it will use choreo for translation of the swerve and the auto aim for rotation but this is stationary)
        return new AutoShoot(drivetrain, shooter, hopper, intake, leds, false, Optional.empty());
    }

    private void addSideOptions(SendableChooser<String> sideChooser) {
        sideChooser.addOption("Left", "L");
        sideChooser.addOption("Right", "R");
        sideChooser.addOption("Mid", "M");
        // sideChooser.addOption("Mid Left", "ML");
        // sideChooser.addOption("Mid Right", "MR");
        sideChooser.setDefaultOption("None", "None");
    }

    private void addIntakeOptions(SendableChooser<String> intakeChooser) {
        intakeChooser.addOption("Line", "LineIntake");
        intakeChooser.addOption("Far", "FarIntake");
        intakeChooser.addOption("Mid", "MidIntake");
        intakeChooser.addOption("Close", "CloseIntake");
        intakeChooser.addOption("Outer Sweep", "OuterIntake");
        intakeChooser.addOption("Inner Sweep", "InnerIntake");
        //this option is only for middle and middle right autos
        // intakeChooser.addOption("Outpost (for middle auto)", "OutpostIntake");
        intakeChooser.setDefaultOption("None", "None");
    }

    private void addShootOptions(SendableChooser<String> shootChooser) {
        shootChooser.addOption("Shoot", "Shoot");
        // shootChooser.addOption("Shoot Bump", "ShootBump");
        //shootChooser.addOption("Shoot and Climb", "Climb");
        shootChooser.setDefaultOption("None", "None");
    }
}
