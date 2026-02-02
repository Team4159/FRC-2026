package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private final Drivetrain drivetrain;

    public AutoRoutines(AutoFactory factory, Drivetrain drivetrain) {
        m_factory = factory;
        this.drivetrain = drivetrain;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("SimplePath");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    public AutoRoutine leftAuto(){
        final AutoRoutine routine = m_factory.newRoutine("leftAuto");
        final AutoTrajectory leftPath1 = routine.trajectory("Left1");
        final AutoTrajectory leftPath2 = routine.trajectory("Left2");

        routine.active().onTrue(
            leftPath1.resetOdometry().andThen(leftPath1.cmd())
            .andThen(new InstantCommand(() -> drivetrain.setAutoPathAutoAimMode(true)))
            .andThen(leftPath2.cmd())
            .andThen(new InstantCommand(() -> drivetrain.setAutoPathAutoAimMode(false)))
        );

        leftPath1.atTime("enableAutoAim").onTrue(new InstantCommand(() -> drivetrain.setAutoPathAutoAimMode(true)));
        leftPath1.atTime("disableAutoAim").onTrue(new InstantCommand(() -> drivetrain.setAutoPathAutoAimMode(false)));
        
        return routine;
    }

    public AutoRoutine rightAuto() {
        final AutoRoutine routine = m_factory.newRoutine("rightAuto");
        final AutoTrajectory rightPath1 = routine.trajectory("Right1");
        final AutoTrajectory rightPath2 = routine.trajectory("Right2");
        
        routine.active().onTrue(
            rightPath1.resetOdometry().andThen(rightPath1.cmd())
            .andThen(new InstantCommand(() -> drivetrain.setAutoPathAutoAimMode(true)))
            .andThen(rightPath2.cmd())
            .andThen(new InstantCommand(() -> drivetrain.setAutoPathAutoAimMode(false)))
        );
        return routine;
    }
}