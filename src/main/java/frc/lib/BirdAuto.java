package frc.lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Optional;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BirdAuto {
    private static class FieldDimensions {
        private static final Distance kRobotBumperSize = Inches.of(32.0);
        private static final Distance kRobotChassisSize = Inches.of(32.0);

        private static final Distance kFieldHeight = Inches.of(317.69);
        private static final Distance kFieldMiddleY = kFieldHeight.div(2.0);
        private static final Distance kTowerWidth = Inches.of(3.51);
        private static final Distance kTowerX = Inches.of(40).plus(kTowerWidth.div(2.0));
        private static final Distance kOutpostWidth = Inches.of(49.84);

        private static final Distance kAllianceMiddleY = kFieldMiddleY;
        private static final Distance kAllianceMiddleX = Inches.of(100.0);

        private static final Distance kNeutralIntakeY = Inches.of(290.0);

        private static final Distance kDepotX = kRobotBumperSize.div(2.0);
        private static final Distance kDepotY = kFieldHeight.minus(Inches.of(82.84));
        private static final Distance kDepotEntryX = kDepotX.plus(Inches.of(30.0));
        private static final Distance kDepotLeftY = kDepotY.plus(Inches.of(40.0));
        private static final Distance kDepotRightY = kDepotY.minus(Inches.of(40.0));

        private static final Distance kTrenchX = Inches.of(182.11);
        private static final Distance kTrenchAllianceX = kTrenchX.minus(Inches.of(45.0));
        private static final Distance kTrenchNeutralX = kTrenchX.plus(Inches.of(45.0));
        private static final Distance kTrenchLeftY = Inches.of(292.69 - 3);
        private static final Distance kTrenchRightY = Inches.of(25 + 3);

        private static final Distance kClimbFrontX = kTowerX.plus(kRobotChassisSize.div(2.0));
        private static final Distance kClimbUnderX = kClimbFrontX.minus(Inches.of(16.0));
        private static final Distance kClimbEntryX = kClimbFrontX.plus(Inches.of(25.0));
        private static final Distance kClimbMiddleY = Inches.of(147.47);
        private static final Distance kClimbLeftY = kClimbMiddleY.plus(Inches.of(2));
        private static final Distance kClimbRightY = kClimbMiddleY.minus(Inches.of(2));
    }
    
    private static enum FieldSetpoint {
        OUTPOST(FieldDimensions.kRobotBumperSize.div(2.0), FieldDimensions.kOutpostWidth.div(2.0),
                Rotation2d.kZero),
        DEPOT_LEFT(FieldDimensions.kDepotX, FieldDimensions.kDepotLeftY,
                Rotation2d.kCW_90deg),
        DEPOT_RIGHT(FieldDimensions.kDepotX, FieldDimensions.kDepotRightY,
                Rotation2d.kCCW_90deg),
        DEPOT_LEFT_ENTRY(FieldDimensions.kDepotEntryX,
                FieldDimensions.kDepotLeftY,
                Rotation2d.kCW_90deg),
        DEPOT_RIGHT_ENTRY(FieldDimensions.kDepotEntryX,
                FieldDimensions.kDepotRightY,
                Rotation2d.kCCW_90deg),
        ALLIANCE_LEFT(FieldDimensions.kAllianceMiddleX, FieldDimensions.kAllianceMiddleY.plus(Inches.of(75)),
                new Rotation2d()),
        ALLIANCE_MIDDLE(FieldDimensions.kAllianceMiddleX, FieldDimensions.kAllianceMiddleY,
                new Rotation2d()),
        ALLIANCE_RIGHT(FieldDimensions.kAllianceMiddleX, FieldDimensions.kAllianceMiddleY.minus(Inches.of(75)),
                new Rotation2d()),
        NEUTRAL_LEFT_OUTER(FieldDimensions.kAllianceMiddleX.plus(Inches.of(120)), FieldDimensions.kNeutralIntakeY,
                Rotation2d.kCW_90deg),
        NEUTRAL_LEFT_INNER(FieldDimensions.kAllianceMiddleX.plus(Inches.of(10)), FieldDimensions.kNeutralIntakeY,
                Rotation2d.kCW_90deg),
        NEUTRAL_RIGHT_OUTER(FieldDimensions.kAllianceMiddleX.minus(Inches.of(120)), FieldDimensions.kNeutralIntakeY,
                Rotation2d.kCCW_90deg),
        NEUTRAL_RIGHT_INNER(FieldDimensions.kAllianceMiddleX.minus(Inches.of(10)), FieldDimensions.kNeutralIntakeY,
                Rotation2d.kCCW_90deg),
        TRENCH_LEFT_ALLIANCE(FieldDimensions.kTrenchAllianceX,
                FieldDimensions.kTrenchLeftY,
                new Rotation2d()),
        TRENCH_LEFT_NEUTRAL(FieldDimensions.kTrenchNeutralX,
                FieldDimensions.kTrenchLeftY,
                new Rotation2d()),
        TRENCH_RIGHT_ALLIANCE(FieldDimensions.kTrenchAllianceX,
                FieldDimensions.kTrenchRightY,
                new Rotation2d()),
        TRENCH_RIGHT_NEUTRAL(FieldDimensions.kTrenchNeutralX,
                FieldDimensions.kTrenchRightY,
                Rotation2d.kZero),
        CLIMB_LEFT(FieldDimensions.kClimbFrontX, FieldDimensions.kClimbLeftY,
                Rotation2d.k180deg),
        CLIMB_MIDDLE(FieldDimensions.kClimbUnderX, FieldDimensions.kClimbMiddleY,
                Rotation2d.k180deg),
        CLIMB_RIGHT(FieldDimensions.kClimbFrontX, FieldDimensions.kClimbRightY,
                Rotation2d.k180deg),
        CLIMB_LEFT_ENTRY(FieldDimensions.kClimbEntryX,
                FieldDimensions.kClimbLeftY,
                Rotation2d.k180deg),
        CLIMB_MIDDLE_ENTRY(
                FieldDimensions.kClimbEntryX,
                FieldDimensions.kClimbMiddleY,
                Rotation2d.k180deg),
        CLIMB_RIGHT_ENTRY(
                FieldDimensions.kClimbEntryX,
                FieldDimensions.kClimbRightY,
                Rotation2d.k180deg);

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

    private boolean pathFinished;
    private APTarget target;
    private FieldGoal[] goals;
    private int goalProgress;
    private Setpoint[] setpoints;
    private int setpointProgress;

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
        Autopilot autopilot = target == null ? alignmentAutopilot
                : (target.getVelocity() == 0 ? alignmentAutopilot : cruiseAutopilot);
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
                setpoints = setpointFactory(goals[goalProgress],
                        setpoints.length == 0 ? new Setpoint(FieldGoal.NONE, currentPose, Optional.empty(),
                                MetersPerSecond.of(0.0), true)
                                : setpoints[setpoints.length - 1],
                        cruiseSpeed);
            }

            Setpoint setpoint = setpoints[setpointProgress];
            Pose2d desiredPose = alliance == Alliance.Blue ? setpoint.pose
                    : FieldUtil.flipPoseToOtherAlliance(setpoint.pose);
            Optional<Rotation2d> entryAngle = setpoint.entryAngle;
            LinearVelocity targetVelocity = setpoint.targetVelocity;
            translationOnly = setpoint.translationOnly;

            target = new APTarget(desiredPose)
                    .withVelocity(targetVelocity.in(MetersPerSecond));
            if (entryAngle.isPresent()) {
                target = target.withEntryAngle(alliance == Alliance.Blue ? entryAngle.get()
                        : entryAngle.get().rotateBy(Rotation2d.k180deg));
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

    // TODO: tune dynamic entry angles to smooth out paths
    private Setpoint[] setpointFactory(FieldGoal goal, Setpoint lastSetpoint, LinearVelocity cruiseSpeed) {
        boolean lastOnRight = FieldUtil.isPoseOnRight(lastSetpoint.pose);
        boolean lastOnLeft = !lastOnRight;
        boolean lastInAlliance = FieldUtil.isPoseInAllianceZone(Alliance.Blue, lastSetpoint.pose);
        boolean lastInNeutral = !lastInAlliance;
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
                if (lastSetpoint.goal == FieldGoal.DEPOT_RIGHT) {
                    yield new Setpoint[] {
                            new Setpoint(goal,
                                    FieldSetpoint.DEPOT_LEFT.pose.rotateAround(
                                            FieldSetpoint.DEPOT_LEFT.pose.getTranslation(), Rotation2d.k180deg),
                                    Optional.empty(),
                                    MetersPerSecond.of(0.0),
                                    false)
                    };
                }
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.DEPOT_LEFT_ENTRY.pose,
                                Optional.empty(),
                                cruiseSpeed,
                                false),
                        new Setpoint(goal, FieldSetpoint.DEPOT_LEFT.pose,
                                Optional.empty(),
                                MetersPerSecond.of(0.0),
                                false)
                };
            }
            case DEPOT_RIGHT: {
                if (lastSetpoint.goal == FieldGoal.DEPOT_RIGHT) {
                    yield new Setpoint[] {
                            new Setpoint(goal,
                                    FieldSetpoint.DEPOT_RIGHT.pose.rotateAround(
                                            FieldSetpoint.DEPOT_RIGHT.pose.getTranslation(), Rotation2d.k180deg),
                                    Optional.empty(),
                                    MetersPerSecond.of(0.0),
                                    false)
                    };
                }
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.DEPOT_RIGHT_ENTRY.pose,
                                Optional.empty(),
                                cruiseSpeed,
                                false),
                        new Setpoint(goal, FieldSetpoint.DEPOT_RIGHT.pose,
                                Optional.empty(),
                                MetersPerSecond.of(0.0),
                                false)
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
                Translation2d lastToNextDifference = FieldSetpoint.ALLIANCE_MIDDLE.pose.getTranslation()
                        .minus(lastSetpoint.pose.getTranslation());
                Rotation2d lastToNextAngle = new Rotation2d(
                        -Math.atan2(lastToNextDifference.getY(), lastToNextDifference.getX()));
                double angleFromVertical = ((90 - lastToNextAngle.getDegrees()) % 360 + 180) % 360 - 180;
                Rotation2d entryAngle = new Rotation2d(Units.degreesToRadians(90 - angleFromVertical * 2 + 180));
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.ALLIANCE_MIDDLE.pose,
                                Optional.of(entryAngle),
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
                                Optional.empty(),
                                MetersPerSecond.of(0.0),
                                false)
                };
            }
            case NEUTRAL_LEFT_OUTER: {
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.NEUTRAL_LEFT_OUTER.pose,
                                Optional.empty(),
                                MetersPerSecond.of(0.0),
                                false)
                };
            }
            case NEUTRAL_RIGHT_INNER: {
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.NEUTRAL_RIGHT_INNER.pose,
                                Optional.empty(),
                                MetersPerSecond.of(0.0),
                                false)
                };
            }
            case NEUTRAL_RIGHT_OUTER: {
                yield new Setpoint[] {
                        new Setpoint(goal, FieldSetpoint.NEUTRAL_RIGHT_OUTER.pose,
                                Optional.empty(),
                                MetersPerSecond.of(0.0),
                                false)
                };
            }
            case TRENCH_LEFT: {
                var setpoints = new ArrayList<Setpoint>();
                if (lastInAlliance) {
                    if (lastOnRight) {
                        setpoints.add(new Setpoint(goal, FieldSetpoint.ALLIANCE_MIDDLE.pose,
                                Optional.of(Rotation2d.kCCW_90deg), cruiseSpeed, true));
                    }
                    Collections.addAll(setpoints, new Setpoint[] {
                            new Setpoint(goal, FieldSetpoint.TRENCH_LEFT_ALLIANCE.pose,
                                    Optional.of(new Rotation2d(Degrees.of(30))),
                                    cruiseSpeed,
                                    true),
                            new Setpoint(goal, FieldSetpoint.TRENCH_LEFT_NEUTRAL.pose,
                                    Optional.of(new Rotation2d(Degrees.of(-15))),
                                    cruiseSpeed,
                                    true)
                    });
                } else if (lastInNeutral) {
                    Collections.addAll(setpoints, new Setpoint[] {
                            new Setpoint(goal, FieldSetpoint.TRENCH_LEFT_NEUTRAL.pose,
                                    Optional.of(new Rotation2d(Degrees.of(180 - 30))),
                                    cruiseSpeed,
                                    true),
                            new Setpoint(goal, FieldSetpoint.TRENCH_LEFT_ALLIANCE.pose,
                                    Optional.of(new Rotation2d(Degrees.of(180 + 15))),
                                    cruiseSpeed,
                                    true)
                    });
                }
                yield setpoints.toArray(Setpoint[]::new);
            }
            case TRENCH_RIGHT: {
                var setpoints = new ArrayList<Setpoint>();
                if (lastInAlliance) {
                    if (lastOnLeft) {
                        setpoints.add(new Setpoint(goal, FieldSetpoint.ALLIANCE_MIDDLE.pose,
                                Optional.of(Rotation2d.kCW_90deg), cruiseSpeed, true));
                    }
                    Collections.addAll(setpoints, new Setpoint[] {
                            new Setpoint(goal, FieldSetpoint.TRENCH_RIGHT_ALLIANCE.pose,
                                    Optional.of(new Rotation2d(Degrees.of(-30))),
                                    cruiseSpeed,
                                    true),
                            new Setpoint(goal, FieldSetpoint.TRENCH_RIGHT_NEUTRAL.pose,
                                    Optional.of(new Rotation2d(Degrees.of(15))),
                                    cruiseSpeed,
                                    true)
                    });
                } else if (lastInNeutral) {
                    Collections.addAll(setpoints, new Setpoint[] {
                            new Setpoint(goal, FieldSetpoint.TRENCH_RIGHT_NEUTRAL.pose,
                                    Optional.of(new Rotation2d(Degrees.of(180 + 30))),
                                    cruiseSpeed,
                                    true),
                            new Setpoint(goal, FieldSetpoint.TRENCH_RIGHT_ALLIANCE.pose,
                                    Optional.of(new Rotation2d(Degrees.of(180 - 15))),
                                    cruiseSpeed,
                                    true)
                    });
                }
                yield setpoints.toArray(Setpoint[]::new);
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
