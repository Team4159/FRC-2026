package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
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
            .andThen(leftPath2.cmd())
        );
        return routine;
    }
}