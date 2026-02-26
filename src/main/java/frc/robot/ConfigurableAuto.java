package frc.robot;

import java.util.ArrayList;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoAim;
import frc.robot.lib.InstantCommandRunWhenDisabled;
import frc.robot.lib.PoseTrajectory;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;

public class ConfigurableAuto {
    private SendableChooser<String> sideChooser, intakeChooser1, shootChooser1, intakeChooser2, shootChooser2;

    private Field2d generatedRoutineDisplay = new Field2d();

    private final AutoFactory factory;
    private final Drivetrain drivetrain;
    private final Shooter shooter;
    //TODO: implement intake
    private final Intake intake;
    private final Hopper hopper;
    private final LEDs leds;

    private AutoRoutine generatedRoutine;

    public ConfigurableAuto(AutoFactory factory, Drivetrain drivetrain, Shooter shooter, Intake intake, Hopper hopper, LEDs leds) {
        //auto factory
        this.factory = factory;

        //subsystems
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;
        this.leds = leds;

        //sendable choosers
        sideChooser = new SendableChooser<>();
        intakeChooser1 = new SendableChooser<>();
        shootChooser1 = new SendableChooser<>();
        intakeChooser2 = new SendableChooser<>();
        shootChooser2 = new SendableChooser<>();

        displayChoosers();
    }

    /** displays sendable chooser options for configuration and the generate button */
    public void displayChoosers(){
        //side chooser
        sideChooser.addOption("Left", "L");
        sideChooser.addOption("Right", "R");

        //intake chooser 1
        intakeChooser1.addOption("Far", "FarIntake");
        intakeChooser1.addOption("Mid", "MidIntake");
        intakeChooser1.addOption("Close", "CloseIntake");

        //shoot chooser 1
        shootChooser1.addOption("Shoot", "Shoot");
        shootChooser1.addOption("Shoot and Climb Left", "LClimb");
        shootChooser1.addOption("Shoot and Climb Right", "RClimb");

        //intake chooser 2
        intakeChooser2.addOption("Far", "FarIntake");
        intakeChooser2.addOption("Mid", "MidIntake");
        intakeChooser2.addOption("Close", "CloseIntake");

        //shoot chooser 2
        shootChooser2.addOption("Shoot", "Shoot");
        shootChooser2.addOption("Shoot and Climb Left", "LClimb");
        shootChooser2.addOption("Shoot and Climb Right", "RClimb");

        //display on smartdashboard -> elastic
        SmartDashboard.putData("side", sideChooser);
        SmartDashboard.putData("Intake 1", intakeChooser1);
        SmartDashboard.putData("Shoot 1", shootChooser1);
        SmartDashboard.putData("Intake 2", intakeChooser2);
        SmartDashboard.putData("Shoot 2", shootChooser2);
        SmartDashboard.putData("generate", new InstantCommandRunWhenDisabled(() -> generateRoutine(true)));
    }

    public void test(){
        System.out.println("test");
    }

    private AutoRoutine generateRoutine(boolean display){
        System.out.println("generateroutine");
        final AutoRoutine routine = factory.newRoutine("Generated Auto");

        final String direction = sideChooser.getSelected();
        final String intake1 = intakeChooser1.getSelected();
        final String shoot1 = shootChooser1.getSelected();
        final String intake2 = intakeChooser2.getSelected();
        final String shoot2 = shootChooser2.getSelected();

        final String startToIntake1Name = direction + "StartTo" + direction + intake1;
        final String intake1ToShoot1Name = direction + intake1 + "To" + direction + "Shoot";
        final String shoot1ToIntake2Name = direction + "Shoot" + "To" + direction + intake2;
        final String intake2ToShoot2Name = direction + intake2 + "To" + direction + "Shoot";

        final AutoTrajectory startToIntake1Traj = routine.trajectory(startToIntake1Name);
        final AutoTrajectory intake1ToShoot1Traj = routine.trajectory(intake1ToShoot1Name);
        final AutoTrajectory shoot1ToIntake2Traj = routine.trajectory(shoot1ToIntake2Name);
        final AutoTrajectory intake2ToShoot2Traj = routine.trajectory(intake2ToShoot2Name);

        System.out.println(shoot2);

        if(shoot1.contains("Climb")){
            final AutoTrajectory shoot1ToClimbTraj = routine.trajectory(direction + "ShootTo" + shoot1);
            routine.active().onTrue(
                startToIntake1Traj.resetOdometry().andThen(startToIntake1Traj.cmd())
                .andThen(intake1ToShoot1Traj.cmd())
                .andThen(new ParallelDeadlineGroup(
                    new WaitCommand(AutoConstants.ShootTime), 
                    new AutoAim(drivetrain, shooter, hopper, leds, display)))
                .andThen(shoot1ToClimbTraj.cmd())
            );

            if(display){
                updateField(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToClimbTraj);
            }
        }
        
        else if(shoot2.contains("Climb")){
            final AutoTrajectory shoot2ToClimbTraj = routine.trajectory(direction + "ShootTo" + shoot2);
            System.out.println(shoot2ToClimbTraj);
            routine.active().onTrue(
                startToIntake1Traj.resetOdometry().andThen(startToIntake1Traj.cmd())
                .andThen(intake1ToShoot1Traj.cmd())
                .andThen(new ParallelDeadlineGroup(
                    new WaitCommand(AutoConstants.ShootTime), 
                    new AutoAim(drivetrain, shooter, hopper, leds, display)))
                .andThen(shoot1ToIntake2Traj.cmd())
                .andThen(intake2ToShoot2Traj.cmd())
                .andThen(new ParallelDeadlineGroup(
                    new WaitCommand(AutoConstants.ShootTime), 
                    new AutoAim(drivetrain, shooter, hopper, leds, display)))
                .andThen(shoot2ToClimbTraj.cmd())
            );

            if(display){
                updateField(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToIntake2Traj, intake2ToShoot2Traj, shoot2ToClimbTraj);
            }
        }

        else{
            routine.active().onTrue(
                startToIntake1Traj.resetOdometry().andThen(startToIntake1Traj.cmd())
                .andThen(intake1ToShoot1Traj.cmd())
                .andThen(new ParallelDeadlineGroup(
                    new WaitCommand(AutoConstants.ShootTime), 
                    new AutoAim(drivetrain, shooter, hopper, leds, display)))
                .andThen(shoot1ToIntake2Traj.cmd())
                .andThen(intake2ToShoot2Traj.cmd())
                .andThen(new ParallelDeadlineGroup(
                    new WaitCommand(AutoConstants.ShootTime), 
                    new AutoAim(drivetrain, shooter, hopper, leds, display)))
            );

            if(display){
                updateField(startToIntake1Traj, intake1ToShoot1Traj, shoot1ToIntake2Traj, intake2ToShoot2Traj);
            }
        }

        startToIntake1Traj.atTime("enableAutoAim").onTrue(new InstantCommand(() -> drivetrain.setAutoPathAutoAimMode(true)));
        startToIntake1Traj.atTime("disableAutoAim").onTrue(new InstantCommand(() -> drivetrain.setAutoPathAutoAimMode(false)));
        
        //store the routine so don't need to generate at the start of auto
        generatedRoutine = routine;
        return routine;
    }

    /** returns the generated routine if it exists otherwise it generates the routine and returns it
     */
    public AutoRoutine getRoutine(){
        if(generatedRoutine == null) return generateRoutine(true);
        return generatedRoutine;
    }

    /**
     * displays an autoroutine on smartdashboard
     * @param trajectories the Choreo AutoTrajectories that make up the routine desired to be displayed
     */
    public void updateField(AutoTrajectory... trajectories){
        System.out.println("updatefield");
        edu.wpi.first.math.trajectory.Trajectory trajectory = new edu.wpi.first.math.trajectory.Trajectory();
        for(int i = 0; i < trajectories.length; i++){
            Trajectory<SwerveSample> choreoTrajectory = trajectories[i].getRawTrajectory();

            ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
            for(int j = 0; j < choreoTrajectory.getPoses().length; j++){
                poses.add(choreoTrajectory.getPoses()[j]);
            }
            PoseTrajectory pt = new PoseTrajectory(poses);
            trajectory = trajectory.concatenate(pt);
        }
        generatedRoutineDisplay.getObject("traj").setTrajectory(trajectory);
        SmartDashboard.putData("Generated Routine Display", generatedRoutineDisplay);
    }
}
