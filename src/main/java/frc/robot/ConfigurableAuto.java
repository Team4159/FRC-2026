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
    public ConfigurableAuto(AutoFactory factory, Drivetrain drivetrain, Shooter shooter, Intake intake, Hopper hopper, LEDs leds) {
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

        SmartDashboard.putData("Generated Routine Display", generatedRoutineDisplay);

        addChooserOptions();
        displayChoosers();
    }

    /** adds options to the choosers
     * default option is always "none"
     * this system will not stop you from inputting an invalid trajectory (such as left -> outpost)
     * and instead it will just give you an error message during generation
     * last year we updated the options of choosers that came after each time a chooser is updated, but due to how elastic works it never changed the display and made the change in options unclear to the drivers
     */
    private void addChooserOptions(){
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
        //this option is only for middle and middle right autos
        intakeChooser1.addOption("Outpost (for middle auto)", "OutpostIntake");
        intakeChooser1.setDefaultOption("None", "None");

        // shoot chooser 1
        shootChooser1.addOption("Shoot", "Shoot");
        shootChooser1.addOption("Shoot Bump", "ShootBump");
        //shootChooser1.addOption("Shoot and Climb", "Climb");
        shootChooser1.setDefaultOption("None", "None");

        // intake chooser 2
        intakeChooser2.addOption("Line", "LineIntake");
        intakeChooser2.addOption("Far", "FarIntake");
        intakeChooser2.addOption("Mid", "MidIntake");
        intakeChooser2.addOption("Close", "CloseIntake");
        intakeChooser2.setDefaultOption("None", "None");

        // shoot chooser 2
        shootChooser2.addOption("Shoot", "Shoot");
        shootChooser2.addOption("Shoot Bump", "ShootBump");
        //shootChooser2.addOption("Shoot and Climb", "Climb");
        shootChooser2.setDefaultOption("None", "None");

        // climbSideChooser.setDefaultOption("Left", "L");
        // climbSideChooser.addOption("Right", "R");
    }

    /**
     * displays the sendable chooser options for configuration and the generate button
     */
    private void displayChoosers() {
        // display on smartdashboard -> elastic
        SmartDashboard.putData("side", sideChooser);
        SmartDashboard.putData("Intake 1", intakeChooser1);
        SmartDashboard.putData("Shoot 1", shootChooser1);
        SmartDashboard.putData("Intake 2", intakeChooser2);
        SmartDashboard.putData("Shoot 2", shootChooser2);
        //SmartDashboard.putData("climb side", climbSideChooser);
        //the InstantCommandRunWhenDisabled is exactly as it sounds, an instant command (literally copied and pasted from WPILIB) except modified so it can be run when disabled to generate the auto routine
        SmartDashboard.putData("generate", new InstantCommandRunWhenDisabled(() -> generateRoutine(true)));
    }

    /** @param display should the generated trajectory be added to the generatedRoutineDisplay as a trajectory
     * will send elastic notifications on the status of the auto
     * @return an AutoRoutine object of the generated routine
     */
    private AutoRoutine generateRoutine(boolean display) {
        final AutoRoutine routine = factory.newRoutine("Generated Auto");

        final String direction = sideChooser.getSelected();
        
        // if the direction is none return the default routine (does absolutely nothing) and send a special notification to let the drivers know they selected a useless auto (could be good if auto is cooked though)
        if (direction.equals("None")) {
            Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.INFO,
                    "Empty auto generated", "this auto will do absolutely nothing");
            Elastic.sendNotification(notification);
            return routine;
        }

        //get all the chooser results as strings to make things cleaner
        final String intake1 = intakeChooser1.getSelected();
        final String shoot1 = shootChooser1.getSelected();
        final String intake2 = intakeChooser2.getSelected();
        final String shoot2 = shootChooser2.getSelected();
        //final String climbSide = climbSideChooser.getSelected();

        //for auto beach recovery (unused, not enough testing time)
        final AutoRecoverySide autoRecoverySide;
        if (direction.equals("L")) {
            autoRecoverySide = AutoRecoverySide.LEFT;
        } else if (direction.equals("R")) {
            autoRecoverySide = AutoRecoverySide.RIGHT;
        } else {
            autoRecoverySide = AutoRecoverySide.MIDDLE;
        }

        //if the direction has a capital "M" then it is a mid auto (ML, M, or MR)
        if (direction.contains("M")) {
            // check if the 1st intake has "Outpost"
            //if so generate an outpost auto
            // outpost auto
            if (intake1.contains("Outpost")) {
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
                    startToIntakeTraj.resetOdometry()
                        //start -> outpost intake
                        .andThen(startToIntakeTraj.cmd())
                        //outpost intake -> shooting position
                        .andThen(intakeToShootTraj.cmd())
                        //auto aim(autonomous mode is false because the point of autonomous mode is for SOTM it will use choreo for translation of the swerve and the auto aim for rotation but this is stationary)
                        .andThen(new AutoAim(drivetrain, shooter, hopper, intake, leds, false, Optional.empty())));

                //choreo marker behavior
                //used to tell robot when to intake and stop intaking based on markers in the intake trajectories
                startToIntakeTraj.atTime("intake").onTrue(intake.new ChangeStates(IntakeState.DOWN_ON));
                startToIntakeTraj.atTime("stopIntake").onTrue(intake.new ChangeStates(IntakeState.DOWN_OFF));

                //update the display field if the display boolean is true
                if (display) {
                    updateField(startToIntakeTraj, intakeToShootTraj);
                }

                //save the routine in the generatedRoutine member
                generatedRoutine = routine;

                displayGenerationStatus(startToIntakeTraj, intakeToShootTraj);

                return routine;
            }

            final String startToShootName = direction + "StartToShoot";
            //final String shootToClimbName = direction + "ShootTo" + climbSide + "Climb";

            final AutoTrajectory startToShootTraj = routine.trajectory(startToShootName);
            //final AutoTrajectory shootToClimbTraj = routine.trajectory(shootToClimbName);

            routine.active().onTrue(
                //resetOdometry() at the start sets the robot inital position to the start point of the 1st trajectory
                startToShootTraj.resetOdometry()
                    //start -> shooting postion
                    //the regular mid auto just scores the preloads
                    .andThen(startToShootTraj.cmd())
                    //auto aim(autonomous mode is false because the point of autonomous mode is for SOTM it will use choreo for translation of the swerve and the auto aim for rotation but this is stationary)
                    .andThen(new AutoAim(drivetrain, shooter, hopper, intake, leds, false, Optional.empty()))
            // .andThen(shootToClimbTraj.cmd())
            // .andThen(new AutoAlign(drivetrain, towerAlignGoal,
            // primaryRobotRelativeTrigger))
            );

            if (display) {
                updateField(startToShootTraj);
            }

            generatedRoutine = routine;

            displayGenerationStatus(startToShootTraj);

            return routine;
        }

        //create the names of the trajectories from the sendable chooser data concatenated together along with other words like "To" so it matches the names of the choreo trajectories
        final String startToIntake1Name = direction + "StartTo" + direction + intake1;
        final String intake1ToShoot1Name = direction + intake1 + "To" + direction + shoot1;
        final String shoot1ToIntake2Name = direction + shoot1 + "To" + direction + intake2;
        final String intake2ToShoot2Name = direction + intake2 + "To" + direction + shoot2;

        //load the AutoTrajectories using the names
        final AutoTrajectory startToIntake1Traj = routine.trajectory(startToIntake1Name);
        final AutoTrajectory intake1ToShoot1Traj = routine.trajectory(intake1ToShoot1Name);
        final AutoTrajectory shoot1ToIntake2Traj = routine.trajectory(shoot1ToIntake2Name);
        final AutoTrajectory intake2ToShoot2Traj = routine.trajectory(intake2ToShoot2Name);

        //commented climb logic
        // if shoot1 is climb, disregard shoot1tointake2 and intake2toshoot2
        // if (shoot1.contains("Climb")) {
        //     final AutoTrajectory shoot1ToClimbTraj = routine.trajectory(direction + "ShootTo" + climbSide + "Climb");
        //     routine.active().onTrue(
        //             startToIntake1Traj.resetOdometry()
        //                     .andThen(startToIntake1Traj.cmd())
        //                     .andThen(intake1ToShoot1Traj.cmd())
        //                     .andThen(new ParallelDeadlineGroup(
        //                             new WaitCommand(AutoConstants.ShootTime),
        //                             new AutoAim(drivetrain, shooter, hopper, intake, leds, false, Optional.empty())))
        //                     .andThen(shoot1ToClimbTraj.cmd()));

        //     displayGenerationStatus(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToClimbTraj);

        //     if (display) {
        //         updateField(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToClimbTraj);
        //     }
        // }

        // else if (shoot2.contains("Climb")) {
        //     final AutoTrajectory shoot2ToClimbTraj = routine.trajectory(direction + "ShootTo" + climbSide + "Climb");
        //     routine.active().onTrue(
        //             startToIntake1Traj.resetOdometry()
        //                     .andThen(startToIntake1Traj.cmd())
        //                     .andThen(intake1ToShoot1Traj.cmd())
        //                     //.andThen(new AutoRecovery(drivetrain, shooter, intake, AutoRecoveryMode.SWEEP, autoRecoverySide,
        //                             //intake1ToShoot1Traj.getFinalPose().get().getTranslation()))
        //                     .andThen(new ParallelDeadlineGroup(
        //                             new WaitCommand(AutoConstants.ShootTime),
        //                             new AutoAim(drivetrain, shooter, hopper, intake, leds, false, Optional.empty())))
        //                     .andThen(shoot1ToIntake2Traj.cmd())
        //                     .andThen(intake2ToShoot2Traj.cmd())
        //                     // .andThen(new AutoRecovery(drivetrain, shooter, intake, AutoRecoveryMode.SWEEP, autoRecoverySide,
        //                     //         intake2ToShoot2Traj.getFinalPose().get().getTranslation()))
        //                     .andThen(new ParallelDeadlineGroup(
        //                             new WaitCommand(AutoConstants.ShootTime),
        //                             new AutoAim(drivetrain, shooter, hopper, intake, leds, false, Optional.empty())))
        //                     .andThen(shoot2ToClimbTraj.cmd()));

        //     displayGenerationStatus(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToIntake2Traj, intake2ToShoot2Traj,
        //             shoot2ToClimbTraj);

        //     if (display) {
        //         updateField(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToIntake2Traj, intake2ToShoot2Traj,
        //                 shoot2ToClimbTraj);
        //     }
        // } else {
            routine.active().onTrue(
                //resetOdometry() at the start sets the robot inital position to the start point of the 1st trajectory
                startToIntake1Traj.resetOdometry()
                    //start -> intake 1
                    .andThen(startToIntake1Traj.cmd())
                    //intake 1 -> shoot 1
                    .andThen(intake1ToShoot1Traj.cmd())
                    // .andThen(new AutoRecovery(drivetrain, shooter, intake, AutoRecoveryMode.SWEEP, autoRecoverySide,
                    //         intake1ToShoot1Traj.getFinalPose().get().getTranslation()))
                    //timeout on the first auto aim otherwise it will never end (auto aim never finishes) and this needs to finish in order to move on to the next cycle
                    //deadline group terminates auto aim when the time runs out
                    .andThen(new ParallelDeadlineGroup(
                            new WaitCommand(AutoConstants.ShootTime),
                            //auto aim(autonomous mode is false because the point of autonomous mode is for SOTM it will use choreo for translation of the swerve and the auto aim for rotation but this is stationary)
                            new AutoAim(drivetrain, shooter, hopper, intake, leds, false, Optional.empty())))
                    //shoot 1 -> intake 2
                    .andThen(shoot1ToIntake2Traj.cmd())
                    //intake 2 -> shoot 2
                    .andThen(intake2ToShoot2Traj.cmd())
                    // .andThen(new AutoRecovery(drivetrain, shooter, intake, AutoRecoveryMode.SWEEP, autoRecoverySide,
                    //         intake2ToShoot2Traj.getFinalPose().get().getTranslation()))
                    //auto aim(autonomous mode is false because the point of autonomous mode is for SOTM it will use choreo for translation of the swerve and the auto aim for rotation but this is stationary)
                    .andThen(new AutoAim(drivetrain, shooter, hopper, intake, leds, false, Optional.empty())));

            //elastic notifications
            displayGenerationStatus(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToIntake2Traj, intake2ToShoot2Traj);

            //if the display boolean is true display the trajectories
            if (display) {
                updateField(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToIntake2Traj, intake2ToShoot2Traj);
            }
        //}

        //choreo marker behavior
        //used to tell robot when to intake and stop intaking based on markers in the intake trajectories
        startToIntake1Traj.atTime("intake").onTrue(intake.new ChangeStates(IntakeState.DOWN_ON));
        startToIntake1Traj.atTime("stopIntake").onTrue(intake.new ChangeStates(IntakeState.DOWN_OFF));

        shoot1ToIntake2Traj.atTime("intake").onTrue(intake.new ChangeStates(IntakeState.DOWN_ON));
        shoot1ToIntake2Traj.atTime("stopIntake").onTrue(intake.new ChangeStates(IntakeState.DOWN_OFF));
        
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
        //loop through all trajectories
        for (AutoTrajectory trajectory : trajectories) {
            //if a trajectory is empty (it could not be loaded from choreo because it doesnt exist)
            if (trajectory.getRawTrajectory().getPoses().length == 0) {
                //get the name of the invalid trajectory
                String invalidTrajectoryName = trajectory.getRawTrajectory().name();
                //send an error message that says the name of the trajectory, this error is likely caused by an invalid combination of trajectories inputted into the sendable choosers
                Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.ERROR,
                        "Auto Path Generation Failed", invalidTrajectoryName + " is invalid with current settings");
                Elastic.sendNotification(notification);
            }
        }
        return errors;
    }

    /** displays the generation status elastic notification of the given trajectories */
    public void displayGenerationStatus(AutoTrajectory... trajectories) {
        //check all trajectories for errors
        if (checkForErrors(trajectories)) {
            //send an info notification saying the trajectory was generated with errors (checkForErrors() already sends actual error type messages)
            Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.INFO,
                    "Auto Path Generated With Errors", "this just means some paths are missing/invalid");
            Elastic.sendNotification(notification);
        } else {
            //if all the trajectories are good just say auto path generated
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
    public void updateField(AutoTrajectory... autoTrajectories) {
        //make a WPILIB trajectory object (so it can be displayed on a field2d)
        edu.wpi.first.math.trajectory.Trajectory trajectory = new edu.wpi.first.math.trajectory.Trajectory();
        //loop though all the trajectories (these are not WPILIB trajectories but Choreo AutoTrajectories)
        for (int i = 0; i < autoTrajectories.length; i++) {
            //get the raw trajectories from choreo (which is a Choreo class confusingly also called Trajectory 😭)
            Trajectory<SwerveSample> choreoTrajectory = autoTrajectories[i].getRawTrajectory();

            //loop through the choreo trajectory to get an ArrayList of Pose2ds
            ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
            for (int j = 0; j < choreoTrajectory.getPoses().length; j++) {
                poses.add(choreoTrajectory.getPoses()[j]);
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

}
