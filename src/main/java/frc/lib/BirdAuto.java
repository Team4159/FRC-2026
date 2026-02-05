package frc.lib;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.Collections;

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
        OUTPOST(),
        DEPOT_LEFT(FieldMeasurementConstants.kDepotX, FieldMeasurementConstants.kDepotLeftY,
                new Rotation2d()),
        DEPOT_RIGHT(FieldMeasurementConstants.kDepotX, FieldMeasurementConstants.kDepotRightY,
                new Rotation2d()),
        DEPOT_LEFT_ENTRY(FieldMeasurementConstants.kDepotEntryX,
                FieldMeasurementConstants.kDepotLeftY,
                new Rotation2d()),
        DEPOT_RIGHT_ENTRY(FieldMeasurementConstants.kDepotEntryX,
                FieldMeasurementConstants.kDepotRightY,
                new Rotation2d()),
        ALLIANCE_LEFT(),
        ALLIANCE_MIDDLE(),
        ALLIANCE_RIGHT(),
        NEUTRAL_LEFT_OUTER(),
        NEUTRAL_LEFT_INNER(),
        NEUTRAL_RIGHT_OUTER(),
        NEUTRAL_RIGHT_INNER(),
        TRENCH_LEFT_ALLIANCE(FieldMeasurementConstants.kTrenchAllianceX,
                FieldMeasurementConstants.kTrenchLeftY,
                new Rotation2d()),
        TRENCH_LEFT_NEUTRAL(FieldMeasurementConstants.kTrenchNeutralX,
                FieldMeasurementConstants.kTrenchLeftY,
                new Rotation2d()),
        TRENCH_RIGHT_ALLIANCE(FieldMeasurementConstants.kTrenchAllianceX,
                FieldMeasurementConstants.kTrenchRightY,
                new Rotation2d()),
        TRENCH_RIGHT_NEUTRAL(FieldMeasurementConstants.kTrenchNeutralX,
                FieldMeasurementConstants.kTrenchRightY,
                new Rotation2d()),
        CLIMB_LEFT(FieldMeasurementConstants.kClimbX, FieldMeasurementConstants.kClimbLeftY,
                new Rotation2d()),
        CLIMB_MIDDLE(FieldMeasurementConstants.kClimbX, FieldMeasurementConstants.kClimbMiddleY,
                new Rotation2d()),
        CLIMB_RIGHT(FieldMeasurementConstants.kClimbX, FieldMeasurementConstants.kClimbRightY,
                new Rotation2d()),
        CLIMB_LEFT_ENTRY(FieldMeasurementConstants.kClimbEntryX,
                FieldMeasurementConstants.kClimbLeftY,
                new Rotation2d()),
        CLIMB_MIDDLE_ENTRY(
                FieldMeasurementConstants.kClimbEntryX,
                FieldMeasurementConstants.kClimbMiddleY,
                new Rotation2d()),
        CLIMB_RIGHT_ENTRY(
                FieldMeasurementConstants.kClimbEntryX,
                FieldMeasurementConstants.kClimbRightY,
                new Rotation2d());

        public final Pose2d pose;

        private FieldSetpoint(Distance x, Distance y, Rotation2d rotation) {
            this.pose = new Pose2d(x, y, rotation);
        }

        private FieldSetpoint() {
            this.pose = new Pose2d();
        }
    }

    public static enum FieldGoal {
        OUTPOST,
        DEPOT_LEFT,
        DEPOT_RIGHT,
        ALLIANCE_LEFT,
        ALLIANCE_MIDDLE,
        ALLIANCE_RIGHT,
        NEUTRAL_LEFT_OUTER,
        NEUTRAL_LEFT_INNER,
        NEUTRAL_RIGHT_OUTER,
        NEUTRAL_RIGHT_INNER,
        TRENCH_LEFT,
        TRENCH_RIGHT,
        CLIMB_LEFT,
        CLIMB_MIDDLE,
        CLIMB_RIGHT;
    }

    @FunctionalInterface
    // for autofilling
    private interface SetpointGenerator {
        Setpoint[] generate(Setpoint previousSetpoint);
    }

    private static record Setpoint(FieldGoal goal, Pose2d pose, Rotation2d entryAngle, LinearVelocity targetVelocity,
            boolean translationOnly) {
    }

    private final Setpoint[] setpoints;
    private int pathProgress;
    private boolean pathFinished;

    public BirdAuto(FieldGoal[] goals) {
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
            if (pathProgress >= setpoints.length) {
                pathFinished = true;
                return;
            }
        }
        // follow
        Setpoint setpoint = setpoints[pathProgress];
        Pose2d pose = alliance == Alliance.Blue ? setpoint.pose
                : PoseUtil.flipPoseToOtherAlliance(setpoint.pose);
        Rotation2d entryAngle = setpoint.entryAngle;
        LinearVelocity targetVelocity = setpoint.targetVelocity;
        boolean translationOnly = setpoint.translationOnly;
    }

    public void reset() {
        pathProgress = 0;
        pathFinished = false;
    }

    private Setpoint[] getSetpointsFromGoals(FieldGoal[] goals) {
        ArrayList<Setpoint> setpoints = new ArrayList<>();
        Setpoint lastSetpoint = null;
        for (FieldGoal goal : goals) {
            Setpoint[] nextSetpoints = setpointFactory(goal, lastSetpoint);
            if (nextSetpoints.length > 0) {
                lastSetpoint = nextSetpoints[nextSetpoints.length - 1];
            }
            Collections.addAll(setpoints, nextSetpoints);
        }
        return setpoints.toArray(Setpoint[]::new);
    }

    private Setpoint[] setpointFactory(FieldGoal goal, Setpoint lastSetpoint) {
        return switch (goal) {
            case OUTPOST: {
                yield new Setpoint[] {
                    new Setpoint(goal, new Pose2d(), new Rotation2d(), MetersPerSecond.of(0.0), false)
                };
            }
            case DEPOT_LEFT: {
                if (lastSetpoint.goal == goal) {
                    yield new Setpoint[] { 
                        new Setpoint(goal, FieldSetpoint.DEPOT_LEFT.pose, new Rotation2d(), MetersPerSecond.of(0.0), false)
                    };
                }
                yield new Setpoint[] { 
                    new Setpoint(goal, FieldSetpoint.DEPOT_LEFT_ENTRY.pose, new Rotation2d(), MetersPerSecond.of(0.0), false),
                    new Setpoint(goal, FieldSetpoint.DEPOT_LEFT.pose, new Rotation2d(), MetersPerSecond.of(0.0), false)
                };
            }
            case DEPOT_RIGHT: {
                if (lastSetpoint.goal == goal) {
                    yield new Setpoint[] { 
                        new Setpoint(goal, FieldSetpoint.DEPOT_RIGHT.pose, new Rotation2d(), MetersPerSecond.of(0.0), false)
                    };
                }
                yield new Setpoint[] { 
                    new Setpoint(goal, FieldSetpoint.DEPOT_RIGHT_ENTRY.pose, new Rotation2d(), MetersPerSecond.of(0.0), false),
                    new Setpoint(goal, FieldSetpoint.DEPOT_RIGHT.pose, new Rotation2d(), MetersPerSecond.of(0.0), false)
                };
            }
            case ALLIANCE_LEFT: {
                yield new Setpoint[] { 
                    new Setpoint(goal, FieldSetpoint.ALLIANCE_LEFT.pose, new Rotation2d(), MetersPerSecond.of(0.0), true)
                };
            }
            case ALLIANCE_MIDDLE: {
                yield new Setpoint[] {
                    new Setpoint(goal, FieldSetpoint.ALLIANCE_MIDDLE.pose, new Rotation2d(), MetersPerSecond.of(0.0), true)
                };
            }
            case ALLIANCE_RIGHT: {
                yield new Setpoint[] {
                    new Setpoint(goal, FieldSetpoint.ALLIANCE_RIGHT.pose, new Rotation2d(), MetersPerSecond.of(0.0), true)
                };
            }
            case NEUTRAL_LEFT_INNER: {
                yield new Setpoint[] {
                    new Setpoint(goal, FieldSetpoint.NEUTRAL_LEFT_INNER.pose, new Rotation2d(), MetersPerSecond.of(0.0), false)
                };
            }
            case NEUTRAL_LEFT_OUTER: {
                yield new Setpoint[] {
                    new Setpoint(goal, FieldSetpoint.NEUTRAL_LEFT_OUTER.pose, new Rotation2d(), MetersPerSecond.of(0.0), false)
                };
            }
            case NEUTRAL_RIGHT_INNER: {
                yield new Setpoint[] {
                    new Setpoint(goal, FieldSetpoint.NEUTRAL_RIGHT_INNER.pose, new Rotation2d(), MetersPerSecond.of(0.0), false)
                };
            }
            case NEUTRAL_RIGHT_OUTER: {
                yield new Setpoint[] {
                    new Setpoint(goal, FieldSetpoint.NEUTRAL_RIGHT_OUTER.pose, new Rotation2d(), MetersPerSecond.of(0.0), false)
                };
            }
            case TRENCH_LEFT: {
                if (PoseUtil.isPoseInAllianceZone(Alliance.Blue, lastSetpoint.pose)) {
                    yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.TRENCH_LEFT_ALLIANCE.pose, new Rotation2d(), MetersPerSecond.of(0.0), false),
                        new Setpoint(goal, FieldSetpoint.TRENCH_LEFT_NEUTRAL.pose, new Rotation2d(), MetersPerSecond.of(0.0), false)
                    };
                }
                yield new Setpoint[] {
                    new Setpoint(goal, FieldSetpoint.TRENCH_LEFT_NEUTRAL.pose, new Rotation2d(), MetersPerSecond.of(0.0), false),
                    new Setpoint(goal, FieldSetpoint.TRENCH_LEFT_ALLIANCE.pose, new Rotation2d(), MetersPerSecond.of(0.0), false)
                };
            }
            case TRENCH_RIGHT: {
                if (PoseUtil.isPoseInAllianceZone(Alliance.Blue, lastSetpoint.pose)) {
                    yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.TRENCH_RIGHT_ALLIANCE.pose, new Rotation2d(), MetersPerSecond.of(0.0), true),
                        new Setpoint(goal, FieldSetpoint.TRENCH_RIGHT_NEUTRAL.pose, new Rotation2d(), MetersPerSecond.of(0.0), true)
                    };
                }
                yield new Setpoint[] {
                    new Setpoint(goal, FieldSetpoint.TRENCH_RIGHT_NEUTRAL.pose, new Rotation2d(), MetersPerSecond.of(0.0), true),
                    new Setpoint(goal, FieldSetpoint.TRENCH_RIGHT_ALLIANCE.pose, new Rotation2d(), MetersPerSecond.of(0.0), true)
                };
            }
            case CLIMB_LEFT: {
                yield new Setpoint[] {
                    new Setpoint(goal, FieldSetpoint.CLIMB_LEFT_ENTRY.pose, new Rotation2d(), MetersPerSecond.of(0.0), false),
                    new Setpoint(goal, FieldSetpoint.CLIMB_LEFT.pose, new Rotation2d(), MetersPerSecond.of(0.0), false)
                };
            }
            case CLIMB_MIDDLE: {
                yield new Setpoint[] {
                    new Setpoint(goal, FieldSetpoint.CLIMB_MIDDLE_ENTRY.pose, new Rotation2d(), MetersPerSecond.of(0.0), false),
                    new Setpoint(goal, FieldSetpoint.CLIMB_MIDDLE.pose, new Rotation2d(), MetersPerSecond.of(0.0), false)
                };
            }
            case CLIMB_RIGHT: {
                yield new Setpoint[] {
                    new Setpoint(goal, FieldSetpoint.CLIMB_RIGHT_ENTRY.pose, new Rotation2d(), MetersPerSecond.of(0.0), false),
                    new Setpoint(goal, FieldSetpoint.CLIMB_MIDDLE.pose, new Rotation2d(), MetersPerSecond.of(0.0), false)
                };
            }
            default: {
                yield new Setpoint[] {};
            }
        };
    }
}
