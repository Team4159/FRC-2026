package frc.lib;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BirdAuto {
    private static class FieldSetpointConstants {
        private static final Distance kDepotX = Meters.of(0.0);
        private static final Distance kDepotEntryX = Meters.of(0.0);
        private static final Distance kTrenchAllianceX = Meters.of(0.0);
        private static final Distance kTrenchNeutralX = Meters.of(0.0);
        private static final Distance kClimbX = Meters.of(0.0);
        private static final Distance kClimbEntryX = Meters.of(0.0);
    }

    private static enum FieldSetpoint {
        OUTPOST(new Pose2d()),
        DEPOT_LEFT(new Pose2d(FieldSetpointConstants.kDepotX, Meters.of(0.0), new Rotation2d())),
        DEPOT_RIGHT(new Pose2d(FieldSetpointConstants.kDepotX, Meters.of(0.0), new Rotation2d())),
        DEPOT_LEFT_ENTRY(new Pose2d(FieldSetpointConstants.kDepotEntryX, Meters.of(0.0), new Rotation2d())),
        DEPOT_RIGHT_ENTRY(new Pose2d(FieldSetpointConstants.kDepotEntryX, Meters.of(0.0), new Rotation2d())),
        ALLIANCE_LEFT(new Pose2d()),
        ALLIANCE_MIDDLE(new Pose2d()),
        ALLIANCE_RIGHT(new Pose2d()),
        NEUTRAL_LEFT_OUTER(new Pose2d()),
        NEUTRAL_LEFT_INNER(new Pose2d()),
        NEUTRAL_RIGHT_OUTER(new Pose2d()),
        NEUTRAL_RIGHT_INNER(new Pose2d()),
        TRENCH_LEFT_ALLIANCE(new Pose2d(FieldSetpointConstants.kTrenchAllianceX, Meters.of(0.0), new Rotation2d())),
        TRENCH_LEFT_NEUTRAL(new Pose2d(FieldSetpointConstants.kTrenchNeutralX, Meters.of(0.0), new Rotation2d())),
        TRENCH_RIGHT_ALLIANCE(new Pose2d(FieldSetpointConstants.kTrenchAllianceX, Meters.of(0.0), new Rotation2d())),
        TRENCH_RIGHT_NEUTRAL(new Pose2d(FieldSetpointConstants.kTrenchNeutralX, Meters.of(0.0), new Rotation2d())),
        CLIMB_LEFT(new Pose2d(FieldSetpointConstants.kClimbX, Meters.of(0.0), new Rotation2d())),
        CLIMB_MIDDLE(new Pose2d(FieldSetpointConstants.kClimbX, Meters.of(0.0), new Rotation2d())),
        CLIMB_RIGHT(new Pose2d(FieldSetpointConstants.kClimbX, Meters.of(0.0), new Rotation2d())),
        CLIMB_LEFT_ENTRY(new Pose2d(FieldSetpointConstants.kClimbEntryX, Meters.of(0.0), new Rotation2d())),
        CLIMB_MIDDLE_ENTRY(new Pose2d(FieldSetpointConstants.kClimbEntryX, Meters.of(0.0), new Rotation2d())),
        CLIMB_RIGHT_ENTRY(new Pose2d(FieldSetpointConstants.kClimbEntryX, Meters.of(0.0), new Rotation2d()));

        public final Pose2d setpoint;

        private FieldSetpoint(Pose2d setpoint) {
            this.setpoint = setpoint;
        }
    }

    public static enum FieldGoal {
        OUTPOST((lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.OUTPOST)),
        DEPOT_LEFT((lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.DEPOT_LEFT)),
        DEPOT_RIGHT((lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.DEPOT_RIGHT)),
        DEPOT_LEFT_ENTRY((lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.DEPOT_LEFT_ENTRY)),
        DEPOT_RIGHT_ENTRY((lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.DEPOT_RIGHT_ENTRY)),
        ALLIANCE_LEFT((lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.ALLIANCE_LEFT)),
        ALLIANCE_MIDDLE((lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.ALLIANCE_MIDDLE)),
        ALLIANCE_RIGHT((lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.ALLIANCE_RIGHT)),
        NEUTRAL_LEFT_OUTER((lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.NEUTRAL_LEFT_OUTER)),
        NEUTRAL_LEFT_INNER((lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.NEUTRAL_LEFT_INNER)),
        NEUTRAL_RIGHT_OUTER((lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.NEUTRAL_RIGHT_OUTER)),
        NEUTRAL_RIGHT_INNER((lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.NEUTRAL_RIGHT_INNER)),
        TRENCH_LEFT(
                (lastSetpoint) -> PoseUtil.isPoseInAllianceZone(Alliance.Blue, lastSetpoint)
                        ? goalSetpointBuilder(FieldSetpoint.TRENCH_LEFT_ALLIANCE, FieldSetpoint.TRENCH_LEFT_NEUTRAL)
                        : goalSetpointBuilder(FieldSetpoint.TRENCH_LEFT_NEUTRAL, FieldSetpoint.TRENCH_LEFT_ALLIANCE)),
        TRENCH_RIGHT(
                (lastSetpoint) -> PoseUtil.isPoseInAllianceZone(Alliance.Blue, lastSetpoint)
                        ? goalSetpointBuilder(FieldSetpoint.TRENCH_RIGHT_ALLIANCE, FieldSetpoint.TRENCH_RIGHT_NEUTRAL)
                        : goalSetpointBuilder(FieldSetpoint.TRENCH_RIGHT_NEUTRAL, FieldSetpoint.TRENCH_RIGHT_ALLIANCE)),
        CLIMB_LEFT(
                (lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.CLIMB_LEFT_ENTRY, FieldSetpoint.CLIMB_LEFT)),
        CLIMB_MIDDLE(
                (lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.CLIMB_RIGHT_ENTRY, FieldSetpoint.CLIMB_MIDDLE)),
        CLIMB_RIGHT(
                (lastSetpoint) -> goalSetpointBuilder(FieldSetpoint.CLIMB_RIGHT_ENTRY, FieldSetpoint.CLIMB_RIGHT));

        public final Goal goal;

        private FieldGoal(SetpointGenerator setpointGenerator) {
            this.goal = new Goal(setpointGenerator);
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

    private static record Goal(SetpointGenerator setpointGenerator) {
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
