package frc.lib;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.Collections;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BirdAuto {
    // TODO: finish populating setpoints
    private static class FieldMeasurementConstants {
        private static final Distance kFieldHeight = Inches.of(317.69);
        private static final Distance kFieldMiddleY = kFieldHeight.div(2);
        private static final Distance kDepotX = Inches.of(15.0);
        private static final Distance kDepotEntryX = kDepotX.plus(Inches.of(30.0));
        private static final Distance kDepotLeftY = Inches.of(0.0);
        private static final Distance kDepotRightY = Inches.of(0.0);
        private static final Distance kTrenchX = Inches.of(182.11);
        private static final Distance kTrenchAllianceX = kTrenchX.minus(Inches.of(45.0));
        private static final Distance kTrenchNeutralX = kTrenchX.plus(Inches.of(45.0));
        private static final Distance kTrenchLeftY = Inches.of(292.69);
        private static final Distance kTrenchRightY = Inches.of(25);
        private static final Distance kClimbX = Inches.of(72.0);
        private static final Distance kClimbEntryX = kClimbX.plus(Inches.of(60.0));
        private static final Distance kClimbLeftY = kFieldMiddleY.minus(Inches.of(5));
        private static final Distance kClimbMiddleY = kFieldMiddleY;
        private static final Distance kClimbRightY = kFieldMiddleY.plus(Inches.of(5));
    }

    private static enum FieldSetpoint {
        OUTPOST(new Setpoint(new Pose2d(), new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond),
                false)),
        DEPOT_LEFT(new Setpoint(
                new Pose2d(FieldMeasurementConstants.kDepotX, FieldMeasurementConstants.kDepotLeftY, new Rotation2d()),
                new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), false)),
        DEPOT_RIGHT(new Setpoint(
                new Pose2d(FieldMeasurementConstants.kDepotX, FieldMeasurementConstants.kDepotRightY, new Rotation2d()),
                new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), false)),
        DEPOT_LEFT_ENTRY(new Setpoint(
                new Pose2d(FieldMeasurementConstants.kDepotEntryX, FieldMeasurementConstants.kDepotLeftY,
                        new Rotation2d()),
                new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), false)),
        DEPOT_RIGHT_ENTRY(new Setpoint(
                new Pose2d(FieldMeasurementConstants.kDepotEntryX, FieldMeasurementConstants.kDepotRightY,
                        new Rotation2d()),
                new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), false)),
        ALLIANCE_LEFT(new Setpoint(new Pose2d(), new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond),
                true)),
        ALLIANCE_MIDDLE(new Setpoint(new Pose2d(), new Rotation2d(),
                LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), true)),
        ALLIANCE_RIGHT(new Setpoint(new Pose2d(), new Rotation2d(),
                LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), true)),
        NEUTRAL_LEFT_OUTER(new Setpoint(new Pose2d(), new Rotation2d(),
                LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), false)),
        NEUTRAL_LEFT_INNER(new Setpoint(new Pose2d(), new Rotation2d(),
                LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), false)),
        NEUTRAL_RIGHT_OUTER(new Setpoint(new Pose2d(), new Rotation2d(),
                LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), false)),
        NEUTRAL_RIGHT_INNER(new Setpoint(new Pose2d(), new Rotation2d(),
                LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), false)),
        TRENCH_LEFT_ALLIANCE(new Setpoint(
                new Pose2d(FieldMeasurementConstants.kTrenchAllianceX, FieldMeasurementConstants.kTrenchLeftY,
                        new Rotation2d()),
                new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), true)),
        TRENCH_LEFT_NEUTRAL(new Setpoint(
                new Pose2d(FieldMeasurementConstants.kTrenchNeutralX, FieldMeasurementConstants.kTrenchLeftY,
                        new Rotation2d()),
                new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), true)),
        TRENCH_RIGHT_ALLIANCE(new Setpoint(
                new Pose2d(FieldMeasurementConstants.kTrenchAllianceX, FieldMeasurementConstants.kTrenchRightY,
                        new Rotation2d()),
                new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), true)),
        TRENCH_RIGHT_NEUTRAL(new Setpoint(
                new Pose2d(FieldMeasurementConstants.kTrenchNeutralX, FieldMeasurementConstants.kTrenchRightY,
                        new Rotation2d()),
                new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), true)),
        CLIMB_LEFT(new Setpoint(
                new Pose2d(FieldMeasurementConstants.kClimbX, FieldMeasurementConstants.kClimbLeftY, new Rotation2d()),
                new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), false)),
        CLIMB_MIDDLE(new Setpoint(
                new Pose2d(FieldMeasurementConstants.kClimbX, FieldMeasurementConstants.kClimbMiddleY,
                        new Rotation2d()),
                new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), false)),
        CLIMB_RIGHT(new Setpoint(
                new Pose2d(FieldMeasurementConstants.kClimbX, FieldMeasurementConstants.kClimbRightY, new Rotation2d()),
                new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), false)),
        CLIMB_LEFT_ENTRY(new Setpoint(
                new Pose2d(FieldMeasurementConstants.kClimbEntryX, FieldMeasurementConstants.kClimbLeftY,
                        new Rotation2d()),
                new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), false)),
        CLIMB_MIDDLE_ENTRY(new Setpoint(
                new Pose2d(FieldMeasurementConstants.kClimbEntryX, FieldMeasurementConstants.kClimbMiddleY,
                        new Rotation2d()),
                new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), false)),
        CLIMB_RIGHT_ENTRY(new Setpoint(
                new Pose2d(FieldMeasurementConstants.kClimbEntryX, FieldMeasurementConstants.kClimbRightY,
                        new Rotation2d()),
                new Rotation2d(), LinearVelocity.ofRelativeUnits(0.0, MetersPerSecond), false));

        public final Setpoint setpoint;

        private FieldSetpoint(Setpoint setpoint) {
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
                (lastSetpoint) -> PoseUtil.isPoseInAllianceZone(Alliance.Blue, lastSetpoint.pose)
                        ? goalSetpointBuilder(FieldSetpoint.TRENCH_LEFT_ALLIANCE, FieldSetpoint.TRENCH_LEFT_NEUTRAL)
                        : goalSetpointBuilder(FieldSetpoint.TRENCH_LEFT_NEUTRAL, FieldSetpoint.TRENCH_LEFT_ALLIANCE)),
        TRENCH_RIGHT(
                (lastSetpoint) -> PoseUtil.isPoseInAllianceZone(Alliance.Blue, lastSetpoint.pose)
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

    private static Setpoint[] goalSetpointBuilder(FieldSetpoint... fieldSetpoints) {
        return Stream.of(fieldSetpoints).map((fieldSetpoint) -> fieldSetpoint.setpoint).toArray(Setpoint[]::new);
    }

    @FunctionalInterface
    // for autofilling
    private interface SetpointGenerator {
        Setpoint[] generate(Setpoint previousSetpoint);
    }

    private static record Goal(SetpointGenerator setpointGenerator) {
    }

    private static record Setpoint(Pose2d pose, Rotation2d entryAngle, LinearVelocity targetVelocity,
            boolean translationOnly) {
    }

    private final Setpoint initialSetpoint;
    private final ArrayList<Setpoint> setpoints;
    private int pathProgress;
    private boolean pathFinished;

    public BirdAuto(Setpoint initialSetpoint, ArrayList<Goal> goals) {
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
        Setpoint setpoint = setpoints.get(pathProgress);
        Pose2d pose = alliance == Alliance.Blue ? setpoint.pose : PoseUtil.flipPoseToOtherAlliance(setpoint.pose); 
        Rotation2d entryAngle = setpoint.entryAngle;
        LinearVelocity targetVelocity = setpoint.targetVelocity;
        boolean translationOnly = setpoint.translationOnly;
    }

    public void reset() {
        pathProgress = 0;
        pathFinished = false;
    }

    private ArrayList<Setpoint> getSetpointsFromGoals(ArrayList<Goal> goals) {
        ArrayList<Setpoint> setpoints = new ArrayList<>();
        Setpoint lastSetpoint = null;
        for (Goal goal : goals) {
            Setpoint[] generatedSetpoints = goal.setpointGenerator
                    .generate(lastSetpoint != null ? lastSetpoint : initialSetpoint);
            lastSetpoint = generatedSetpoints[generatedSetpoints.length - 1];
            Collections.addAll(setpoints, generatedSetpoints);
        }
        return setpoints;
    }
}
