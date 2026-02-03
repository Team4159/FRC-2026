package frc.lib;

import java.util.ArrayList;
import java.util.Collections;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BirdAuto {
    private static class FieldSetpointConstants {

    }

    private static enum FieldSetpoint {
        OUTPOST(new Pose2d()),
        DEPOT_LEFT(new Pose2d()),
        DEPOT_RIGHT(new Pose2d()),
        ALLIANCE_LEFT(new Pose2d()),
        ALLIANCE_MIDDLE(new Pose2d()),
        ALLIANCE_RIGHT(new Pose2d()),
        NEUTRAL_LEFT_OUTER(new Pose2d()),
        NEUTRAL_LEFT_INNER(new Pose2d()),
        NEUTRAL_RIGHT_OUTER(new Pose2d()),
        NEUTRAL_RIGHT_INNER(new Pose2d()),
        TRENCH_LEFT_CLOSE(new Pose2d()),
        TRENCH_LEFT_FAR(new Pose2d()),
        TRENCH_RIGHT_CLOSE(new Pose2d()),
        TRENCH_RIGHT_FAR(new Pose2d()),
        CLIMB_LEFT(new Pose2d()),
        CLIMB_MIDDLE(new Pose2d()),
        CLIMB_RIGHT(new Pose2d()),
        CLIMB_LEFT_ENTRY(new Pose2d()),
        CLIMB_MIDDLE_ENTRY(new Pose2d()),
        CLIMB_RIGHT_ENTRY(new Pose2d());

        public final Pose2d setpoint;

        private FieldSetpoint(Pose2d setpoint) {
            this.setpoint = setpoint;
        }
    }

    public static enum FieldGoal {
        CLIMB_LEFT(new Goal(new Pose2d(),
                (previousSet) -> goalSetpointBuilder(FieldSetpoint.CLIMB_LEFT_ENTRY, FieldSetpoint.CLIMB_LEFT))),
        CLIMB_MIDDLE(new Goal(new Pose2d(),
                (lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.CLIMB_RIGHT_ENTRY, FieldSetpoint.CLIMB_MIDDLE))),
        CLIMB_RIGHT(new Goal(new Pose2d(),
                (lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.CLIMB_RIGHT_ENTRY, FieldSetpoint.CLIMB_RIGHT)));

        public final Goal goal;

        private FieldGoal(Goal goal) {
            this.goal = goal;
        }
    }

    private static Pose2d[] goalSetpointBuilder(FieldSetpoint... fieldSetpoints) {
        return Stream.of(fieldSetpoints).map((fieldSetpoint) -> fieldSetpoint.setpoint).toArray(Pose2d[]::new);
    }

    @FunctionalInterface
    // for autofilling
    private interface SetpointGenerator {
        Pose2d[] generate(Pose2d previousSetpoint);
    }

    private static record Goal(Pose2d pose, SetpointGenerator setpointGenerator) {
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
        if (pathFinished) {
            return;
        }
        boolean reachedLastSetpoint = false;
        if (reachedLastSetpoint) {
            pathProgress++;
            if (pathProgress >= setpoints.size()) {
                pathFinished = true;
                return;
            }
        }
        // follow
        Pose2d setpoint = setpoints.get(pathProgress);
        if (alliance == Alliance.Red) {
            setpoint = PoseUtil.flipPoseToOtherAlliance(setpoint);
        }
    }

    public void reset() {
        pathProgress = 0;
        pathFinished = false;
    }

    private ArrayList<Pose2d> getSetpointsFromGoals(ArrayList<Goal> goals) {
        ArrayList<Pose2d> setpoints = new ArrayList<>();
        Pose2d lastSetpoint = null;
        for (Goal goal : goals) {
            Pose2d[] generatedSetpoints = goal.setpointGenerator
                    .generate(lastSetpoint != null ? lastSetpoint : initialSetpoint);
            lastSetpoint = generatedSetpoints[generatedSetpoints.length - 1];
            Collections.addAll(setpoints, generatedSetpoints);
        }
        return setpoints;
    }
}
