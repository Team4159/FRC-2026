package frc.lib;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BirdAuto {
    public static enum FieldSetpoint {
        CLIMB_LEFT(new Pose2d()),
        CLIMB_MIDDLE(new Pose2d()),
        CLIMB_RIGHT(new Pose2d());

        public final Pose2d setpoint;

        private FieldSetpoint(Pose2d setpoint) {
            this.setpoint = setpoint;
        }
    }

    public static enum FieldGoal {
        CLIMB_LEFT(new Goal(new Pose2d(),  lastSetpoint -> new Pose2d[] { FieldSetpoint.CLIMB_LEFT.setpoint })),
        CLIMB_MIDDLE(new Goal(new Pose2d(), lastSetpoint -> new Pose2d[] { FieldSetpoint.CLIMB_LEFT.setpoint })),
        CLIMB_RIGHT(new Goal(new Pose2d(), lastSetpoint -> new Pose2d[] { FieldSetpoint.CLIMB_LEFT.setpoint }));

        public final Goal goal;

        private FieldGoal(Goal goal) {
            this.goal = goal;
        }
    }

    public static record Goal(Pose2d pose, Function<Pose2d, Pose2d[]> setpointGenerator) {
    }

    private final Pose2d initialSetpoint;
    private final ArrayList<Pose2d> setpoints;
    private int pathProgress;
    private boolean pathFinished;

    public BirdAuto(Pose2d initialSetpoint, ArrayList<Goal> goals) {
        this.initialSetpoint = initialSetpoint;
        this.setpoints = getSetpointsFromGoals(goals);
        reset();
    }

    public void followPath(Alliance alliance) {
        if (pathFinished) { return; }
        boolean reachedLastSetpoint = false;
        if (reachedLastSetpoint) {
            pathProgress++;
            if (pathProgress >= setpoints.size()) {
                pathFinished = true;
                return;
            }
        }
        // follow
    }

    public void reset() {
        pathProgress = 0;
        pathFinished = false;
    }

    private ArrayList<Pose2d> getSetpointsFromGoals(ArrayList<Goal> goals) {
        ArrayList<Pose2d> setpoints = new ArrayList<>();
        Pose2d lastSetpoint = null;
        for (Goal goal : goals) {
            Pose2d[] generatedSetpoints = goal.setpointGenerator.apply(lastSetpoint != null ? lastSetpoint : initialSetpoint);
            lastSetpoint = generatedSetpoints[generatedSetpoints.length - 1];
            Collections.addAll(setpoints, generatedSetpoints);
        }
        return setpoints;
    }
}
