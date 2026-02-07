package frc.lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
        private static final Distance kAllianceMiddleX = Inches.of(100);
        private static final Distance kAllianceMiddleY = kFieldMiddleY;
        private static final Distance kTrenchX = Inches.of(182.11);
        private static final Distance kTrenchAllianceX = kTrenchX.minus(Inches.of(45.0));
        private static final Distance kTrenchNeutralX = kTrenchX.plus(Inches.of(45.0));
        private static final Distance kTrenchLeftY = Inches.of(292.69 - 3);
        private static final Distance kTrenchRightY = Inches.of(25 + 3);
        private static final Distance kClimbX = Inches.of(58.0);
        private static final Distance kClimbEntryX = kClimbX.plus(Inches.of(24.0));
        private static final Distance kClimbLeftY = kFieldMiddleY.minus(Inches.of(2));
        private static final Distance kClimbMiddleY = kFieldMiddleY;
        private static final Distance kClimbRightY = kFieldMiddleY.plus(Inches.of(2));
    }

    private static enum FieldSetpoint {
        OUTPOST(Inches.of(15), Inches.of(25), new Rotation2d()),
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
        ALLIANCE_MIDDLE(FieldMeasurementConstants.kAllianceMiddleX, FieldMeasurementConstants.kAllianceMiddleY, new Rotation2d()),
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
        NONE,
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

    public static record AlignmentResult(LinearVelocity velocityX, LinearVelocity velocityY,
            Rotation2d rotationHeading, boolean translationOnly) {
    }

    private static record Setpoint(FieldGoal goal, Pose2d pose, Optional<Rotation2d> entryAngle,
            LinearVelocity targetVelocity,
            boolean translationOnly) {
    }

    private static final AlignmentResult kIdleAlignmentResult = new AlignmentResult(MetersPerSecond.of(0.0),
            MetersPerSecond.of(0.0), Rotation2d.kZero, true);
    private static final Setpoint defaultSetpoint = new Setpoint(FieldGoal.NONE, new Pose2d(), Optional.empty(),
            MetersPerSecond.of(0.0), false);
    
    private boolean pathFinished;
    private APTarget target;
    private FieldGoal[] goals;
    private int goalProgress;
    private Setpoint[] setpoints;
    private int setpointProgress;
    
    // TODO: have separate profiles for cruise and alignment
    public BirdAuto(FieldGoal[] goals) {
        this.goals = goals;
        reset();
    }

    public AlignmentResult calculateAlignment(
        Autopilot cruiseAutopilot, 
        Autopilot alignmentAutopilot,
        Pose2d currentPose, 
        ChassisSpeeds drivetrainRelativeSpeeds,
        Alliance alliance, 
        LinearVelocity cruiseSpeed) {
        if (pathFinished) {
            return kIdleAlignmentResult;
        }
        Autopilot autopilot = target == null ? alignmentAutopilot : (target.getVelocity() == 0 ? alignmentAutopilot : cruiseAutopilot);
        boolean reachedSetpoint = target == null ? true : autopilot.atTarget(currentPose, target);
        boolean translationOnly = false;
        if (reachedSetpoint) {
            setpointProgress++;
            boolean newGoal = false;
            if (setpointProgress >= setpoints.length) {
                newGoal = true;
                goalProgress++;
            }
            if (target == null) {
                newGoal = true;
                goalProgress = 0;
            }
            if (goalProgress >= goals.length) {
                pathFinished = true;
                return kIdleAlignmentResult;
            }
            if (newGoal) {
                setpointProgress = 0;
                setpoints = setpointFactory(goals[goalProgress], Optional.ofNullable(setpoints.length == 0 ? null : setpoints[setpoints.length - 1]), cruiseSpeed);
            }

            Setpoint setpoint = setpoints[setpointProgress];
            Pose2d desiredPose = alliance == Alliance.Blue ? setpoint.pose
                    : PoseUtil.flipPoseToOtherAlliance(setpoint.pose);
            Optional<Rotation2d> entryAngle = setpoint.entryAngle;
            LinearVelocity targetVelocity = setpoint.targetVelocity;
            translationOnly = setpoint.translationOnly;

            System.out.println(setpoint.goal.name());
            target = new APTarget(desiredPose)
                .withVelocity(targetVelocity.in(MetersPerSecond));
            if (entryAngle.isPresent()) {
                target = target.withEntryAngle(entryAngle.get());
            } else {
                target = target.withoutEntryAngle();
            }
        }
        APResult result = autopilot.calculate(currentPose, drivetrainRelativeSpeeds, target);
        return new AlignmentResult(result.vx(), result.vy(), result.targetAngle(), translationOnly);
    }

    public void reset() {
        goalProgress = -1;
        pathFinished = false;
        setpointProgress = -1;
        setpoints = new Setpoint[] {};
    }

    // TODO: add dyanmic entry angles to smooth paths
    private Setpoint[] setpointFactory(FieldGoal goal, Optional<Setpoint> lastSetpoint, LinearVelocity cruiseSpeed) {
        return switch (goal) {
            case OUTPOST: {
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.OUTPOST.pose,
                                Optional.of(new Rotation2d(Degrees.of(180 + 30))),
                                MetersPerSecond.of(0.0),
                                false)
                };
            }
            case DEPOT_LEFT: {
                if (lastSetpoint.isPresent() && lastSetpoint.get().goal == goal) {
                    yield new Setpoint[] {
                            new Setpoint(goal, FieldSetpoint.DEPOT_LEFT.pose,
                                    Optional.of(new Rotation2d()),
                                    MetersPerSecond.of(0.0),
                                    false)
                    };
                }
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.DEPOT_LEFT_ENTRY.pose,
                                Optional.of(new Rotation2d()),
                                MetersPerSecond.of(0.0),
                                false),
                        new Setpoint(goal, FieldSetpoint.DEPOT_LEFT.pose,
                                Optional.of(new Rotation2d()),
                                MetersPerSecond.of(0.0),
                                false)
                };
            }
            case DEPOT_RIGHT: {
                if (lastSetpoint.isPresent() && lastSetpoint.get().goal == goal) {
                    yield new Setpoint[] {
                            new Setpoint(goal, FieldSetpoint.DEPOT_RIGHT.pose,
                                    Optional.of(new Rotation2d()),
                                    MetersPerSecond.of(0.0), false)
                    };
                }
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.DEPOT_RIGHT_ENTRY.pose,
                                Optional.of(new Rotation2d()),
                                MetersPerSecond.of(0.0), false),
                        new Setpoint(goal, FieldSetpoint.DEPOT_RIGHT.pose,
                                Optional.of(new Rotation2d()),
                                MetersPerSecond.of(0.0), false)
                };
            }
            case ALLIANCE_LEFT: {
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.ALLIANCE_LEFT.pose,
                                Optional.empty(),
                                MetersPerSecond.of(0.0),
                                true)
                };
            }
            case ALLIANCE_MIDDLE: {
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.ALLIANCE_MIDDLE.pose,
                                Optional.empty(),
                                MetersPerSecond.of(0.0),
                                true)
                };
            }
            case ALLIANCE_RIGHT: {
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.ALLIANCE_RIGHT.pose,
                                Optional.empty(),
                                MetersPerSecond.of(0.0),
                                true)
                };
            }
            case NEUTRAL_LEFT_INNER: {
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.NEUTRAL_LEFT_INNER.pose,
                                Optional.of(new Rotation2d()),
                                MetersPerSecond.of(0.0),
                                false)
                };
            }
            case NEUTRAL_LEFT_OUTER: {
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.NEUTRAL_LEFT_OUTER.pose,
                                Optional.of(new Rotation2d()),
                                MetersPerSecond.of(0.0),
                                false)
                };
            }
            case NEUTRAL_RIGHT_INNER: {
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.NEUTRAL_RIGHT_INNER.pose,
                                Optional.of(new Rotation2d()),
                                MetersPerSecond.of(0.0),
                                false)
                };
            }
            case NEUTRAL_RIGHT_OUTER: {
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.NEUTRAL_RIGHT_OUTER.pose,
                                Optional.of(new Rotation2d()),
                                MetersPerSecond.of(0.0),
                                false)
                };
            }
            case TRENCH_LEFT: {
                if (!lastSetpoint.isPresent() || PoseUtil.isPoseInAllianceZone(Alliance.Blue, lastSetpoint.get().pose)) {
                    yield new Setpoint[] {
                            new Setpoint(goal, FieldSetpoint.TRENCH_LEFT_ALLIANCE.pose,
                                    Optional.of(new Rotation2d(Degrees.of(30))),
                                    cruiseSpeed,
                                    true),
                            new Setpoint(goal, FieldSetpoint.TRENCH_LEFT_NEUTRAL.pose,
                                    Optional.of(new Rotation2d(Degrees.of(-30))),
                                    cruiseSpeed,
                                    true)
                    };
                }
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.TRENCH_LEFT_NEUTRAL.pose,
                                Optional.of(new Rotation2d(Degrees.of(180 - 30))),
                                cruiseSpeed,
                                true),
                        new Setpoint(goal, FieldSetpoint.TRENCH_LEFT_ALLIANCE.pose,
                                Optional.of(new Rotation2d(Degrees.of(180 + 30))),
                                cruiseSpeed,
                                true)
                };
            }
            case TRENCH_RIGHT: {
                if (!lastSetpoint.isPresent() || PoseUtil.isPoseInAllianceZone(Alliance.Blue, lastSetpoint.get().pose)) {
                    yield new Setpoint[] {
                            new Setpoint(goal, FieldSetpoint.TRENCH_RIGHT_ALLIANCE.pose,
                                    Optional.of(new Rotation2d(Degrees.of(-30))),
                                    cruiseSpeed,
                                    true),
                            new Setpoint(goal, FieldSetpoint.TRENCH_RIGHT_NEUTRAL.pose,
                                    Optional.of(new Rotation2d(Degrees.of(30))),
                                    cruiseSpeed,
                                    true)
                    };
                }
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.TRENCH_RIGHT_NEUTRAL.pose,
                                Optional.of(new Rotation2d(Degrees.of(180 + 30))),
                                cruiseSpeed,
                                true),
                        new Setpoint(goal, FieldSetpoint.TRENCH_RIGHT_ALLIANCE.pose,
                                Optional.of(new Rotation2d(Degrees.of(180 - 30))),
                                cruiseSpeed,
                                true)
                };
            }
            case CLIMB_LEFT: {
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.CLIMB_LEFT_ENTRY.pose,
                                Optional.empty(),
                                cruiseSpeed.times(0.2),
                                false),
                        new Setpoint(goal, FieldSetpoint.CLIMB_LEFT.pose,
                                Optional.empty(),
                                MetersPerSecond.of(0.0),
                                false)
                };
            }
            case CLIMB_MIDDLE: {
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.CLIMB_MIDDLE_ENTRY.pose,
                                Optional.empty(),
                                cruiseSpeed.times(0.2),
                                false),
                        new Setpoint(goal, FieldSetpoint.CLIMB_MIDDLE.pose,
                                Optional.empty(),
                                MetersPerSecond.of(0.0),
                                false)
                };
            }
            case CLIMB_RIGHT: {
                yield new Setpoint[] {
                        new Setpoint(goal,
                                FieldSetpoint.CLIMB_RIGHT_ENTRY.pose,
                                Optional.empty(),
                                cruiseSpeed.times(0.2), 
                                false),
                        new Setpoint(goal, FieldSetpoint.CLIMB_RIGHT.pose,
                                Optional.empty(),
                                MetersPerSecond.of(0.0),
                                false)
                };
            }
            default: {
                yield new Setpoint[] {};
            }
        };
    }
}
