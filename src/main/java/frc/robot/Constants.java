// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DrivetrainConstants.kMaxTranslationSpeed;

import java.util.Map;
import java.util.Set;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class ClimberConstants {
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kP = 0;
        public static final int idClimberOne = 15;
        // public static final int idClimberTwo = 10;

        public static enum ClimberState {
            CLIMB(0.25), STOP(0), DOWN(-0.25);

            public double percentage;

            private ClimberState(double speed) {
                percentage = speed;
            }
        }
    }

    public static class HopperConstants {
        // TODO: set later
        public static final int HopperId = 30;

        public static enum HopperState {
            FEED(1), REVERSE(-1), STOP(0);

            public double percentage;

            private HopperState(double speed) {
                percentage = speed;
            }
        }
    }

    public static class FeederConstants {
        public static final int FeederID = 20; // idk if this port is used yet plz check

        public static enum FeederState {
            FEED(1), UNSTUCKFEEDER(-1), STOP(0);

            public double percentage;

            private FeederState(double speed) {
                percentage = speed;
            }
        }
    }

    public static class IntakeConstants {
        public static final double kAngleI = 1;
        public static final double kAngleD = 0;
        public static final double kAngleP = 40;
        public static final double kAngleG = 0.07;

        // motion magic
        public static final double kFastCruiseVelocity = 200;
        public static final double kFastAcceleration = 500;
        public static final double kFastJerk = 1600;

        public static final double kSlowCruiseVelocity = 1;
        public static final double kSlowAcceleration = 5;
        public static final double kSlowJerk = 1600;

        public static final int kAngleEncoderId = 1;
        public static final int kAngleId = 60; // youre welcome Faye
        public static final int kIntakeSpinId = 70;

        public static final Angle kEncoderOffset = Degrees.of(0);
        //now 25 and 2 because encoder is on the jackshaft now.
        public static final double kMotorToSensorRatio = 25;
        public static final double kSensorToMechanismRatio = 2;

        // motor configs
        public static final CANcoderConfiguration canCoderConfig = new CANcoderConfiguration() {
            {
                MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.9));
                MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
                MagnetSensor.withMagnetOffset(IntakeConstants.kEncoderOffset);
            }
        };

        public static final TalonFXConfiguration angleConfig = new TalonFXConfiguration() {
            {
                Slot0.kP = IntakeConstants.kAngleP;
                Slot0.kI = IntakeConstants.kAngleI;
                Slot0.kD = IntakeConstants.kAngleD;
                Slot0.kG = IntakeConstants.kAngleG;
                Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
                // abs encoder
                Feedback.FeedbackRemoteSensorID = IntakeConstants.kAngleEncoderId;
                Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                Feedback.SensorToMechanismRatio = IntakeConstants.kSensorToMechanismRatio;
                Feedback.RotorToSensorRatio = IntakeConstants.kMotorToSensorRatio;

                CurrentLimits.SupplyCurrentLimitEnable = true;
                CurrentLimits.SupplyCurrentLimit = 20;
            }
        };

        // motion magic
        public static final MotionMagicConfigs kFastMotionMagicConfig = new MotionMagicConfigs() {
            {
                MotionMagicCruiseVelocity = IntakeConstants.kFastCruiseVelocity; // Target cruise velocity of 80 rps
                MotionMagicAcceleration = IntakeConstants.kFastAcceleration; // Target acceleration of 160 rps/s (0.5
                                                                             // seconds)
                MotionMagicJerk = IntakeConstants.kFastJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)
            }
        };

        public static final MotionMagicConfigs kSlowMotionMagicConfig = new MotionMagicConfigs() {
            {
                MotionMagicCruiseVelocity = IntakeConstants.kSlowCruiseVelocity;
                MotionMagicAcceleration = IntakeConstants.kSlowAcceleration;
                MotionMagicJerk = IntakeConstants.kSlowJerk;
            }
        };

        // public static final double kLocationGearRatio = 1.0 / 2.0;
        public static final double kSpinGearRatio = 1.0 / 5.0;

        /** Units: rad/s */
        public static final double kCompressRate = 1;
        public static final double kCompressP = 1;
        public static final double kCompressI = 0;
        public static final double kCompressD = 0;
        public static final ProfiledPIDController compressPID = new ProfiledPIDController(
                kCompressP,
                kCompressI,
                kCompressD,
                new TrapezoidProfile.Constraints(kCompressRate, 1));

        public static enum IntakeState {
            DOWN_ON(Degrees.of(-12), 0.5),
            DOWN_OFF(Degrees.of(-12), 0),
            DOWN_REV(Degrees.of(-12), -0.5),
            UP_OFF(Degrees.of(130), 0),
            BOUNCE_UP(Degrees.of(55), 0),
            STOP(Degrees.of(130), 0);

            public final Angle rotationLocation;
            public final double spinSpeed;

            private IntakeState(Angle location, double speed) {
                rotationLocation = location;
                spinSpeed = speed;
            }
        }
    }

    public static class OperatorConstants {
        public static final int kPrimaryControllerPort = 0;
        public static final int kSecondaryControllerPort = 1;

        // controller joystick constants
        public static final double kPrimaryTranslationDeadband = 0.1;
        public static final double kPrimaryRotationDeadband = 0.1;
        public static final double kPrimaryTranslationExponent = 2.0;
        public static final double kPrimaryRotationExponent = 2.0;
        public static final double kPrimaryTranslationRadius = 0.99;
        public static final double kPrimaryRotationRadius = 0.99;

        public static enum DriveMode {
            TELEOP,
            BRAKE,
            POINT,
            IDLE,
        }

        public static enum DriveFlag {
            SLOW_MODE,
            DRIVE_ASSIST,
            AUTO_BRAKE,
            INTAKE_ASSIST,
            MANUAL_ALIGN,
        }

        // drive assist constants
        public static final Distance kTrenchAssistPassPositionTolerance = Meters.of(0.45);
        public static final double kTrenchAssistApproachInputTolerance = 0.2;
        public static final Distance kTrenchAssistAlignPositionTolerance = Meters.of(0.15);
        public static final double kTrenchAssistAlignStrength = 0.85;
        public static final double kTrenchAssistAlignInfluence = 0.2;
        public static final Distance kTrenchAssistFrontProtrusionExtent = Inches.of(10.0);

        // drive mode constants
        public static final Angle kPrimaryAutoBrakeReachedDesiredAngleTolerance = Degrees.of(5);

        public static final double kPrimarySlowModeTranslationFactor = 0.25;
        public static final double kPrimarySlowModeRotationFactor = 1;

        public static final double kPrimaryIntakeRotationInputDeadzone = 0.2;

        public static final double kPrimaryRadialModeDeadband = 0.2;

        public static final double kPrimaryAlignModeDeadband = 0.65;
        public static final double kPrimaryAlignModeSpeedTranslationFactor = 0.2;
        public static final double kPrimaryAlignModeSpeedRotationFactor = 0.1;
    }

    public static class DrivetrainConstants {
        public static final Distance kDrivetrainSizeX = Inches.of(34.0);
        public static final Distance kDrivetrainSizeY = Inches.of(34.0);

        public static final double kMaxTranslationSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double kMaxRotationSpeed = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        
        public static final double kPointKP = 5;
        public static final double kPointKI = 0.0;
        public static final double kPointKD = 0.0;
        public static final double kPointFeedForward = 0.0;

        public static final double kAimKP = 10;
        public static final double kAimKI = 0.0;
        public static final double kAimKD = 0.0;
        public static final double kAimFeedForward = 0.0;


        public static final Angle AutoAimTolerance = Degrees.of(5);
        public static final double kAutoAimInputMultiplier = 1;

        public static final PhoenixPIDController AutoAimRotationController = new PhoenixPIDController(kAimKP, kAimKI,
                kAimKD);
        static {
            AutoAimRotationController.enableContinuousInput(-Math.PI, Math.PI);
        }

    }

    public static class ShooterConstants {

        public static final double kHoodI = 25;
        public static final double kHoodD = 0;
        public static final double kHoodP = 150;
        public static final double kHoodG = 0.03;
        public static final double kHoodS = 5;
        // public static final double kHoodV = 1.11;
        // public static final double kHoodA = 0.17;
        // abs encoder
        public static final int HoodId = 8;
        public static final int kHoodEncoderID = 2;
        public static final Angle kEncoderOffset = Degrees.of(-255);
        public static final double kSensorToMechanismRatio = 34 / 16;
        public static final double kMotorToSensorRatio = 125;

        public static final double kCruiseVelocity = 40;
        public static final double kAcceleration = 80;
        public static final double kJerk = 1600;

        /** the angle between the center of the shooter and the very edge */
        public static final Angle kHoodAngleOffset = Degrees.of(7.6743605);

        public static final Angle kRestingAngle = Degrees.of(-5.8019605);

        // hood cancoder
        public static final CANcoderConfiguration canCoderConfig = new CANcoderConfiguration() {
            {
                MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
                MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
                MagnetSensor.withMagnetOffset(ShooterConstants.kEncoderOffset);
            }
        };

        // motion magic
        public static final MotionMagicConfigs kHoodMotionMagicConfig = new MotionMagicConfigs() {
            {
                MotionMagicCruiseVelocity = ShooterConstants.kCruiseVelocity; // Target cruise velocity of 80 rps
                MotionMagicAcceleration = ShooterConstants.kAcceleration; // Target acceleration of 160 rps/s (0.5
                                                                          // seconds)
                MotionMagicJerk = ShooterConstants.kJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)
            }
        };
        // hood motor conifg
        public static final TalonFXConfiguration hoodConfig = new TalonFXConfiguration() {
            {
                Slot0.kP = ShooterConstants.kHoodP;
                Slot0.kI = ShooterConstants.kHoodI;
                Slot0.kD = ShooterConstants.kHoodD;
                // Slot0.kS = ShooterConstants.kHoodS;
                // Slot0.kV = ShooterConstants.kHoodV;
                // Slot0.kA = ShooterConstants.kHoodA;
                Slot0.kG = ShooterConstants.kHoodG;
                Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
                MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                MotorOutput.NeutralMode = NeutralModeValue.Brake;

                CurrentLimits.SupplyCurrentLimitEnable = true;
                CurrentLimits.SupplyCurrentLimit = 20;
                // abs encoder
                Feedback.FeedbackRemoteSensorID = ShooterConstants.kHoodEncoderID;
                Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                Feedback.SensorToMechanismRatio = ShooterConstants.kSensorToMechanismRatio;
                Feedback.RotorToSensorRatio = ShooterConstants.kMotorToSensorRatio;
                MotionMagic = kHoodMotionMagicConfig;
            }
        };

        // Shooter Motor Config and PID
        // public static final double kP = 35;
        // public static final double kI = 10;
        public static final double kP = 5;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        //remove if doesnt work
        public static final double kV = 0.25;
        public static final double kA = 2.14;

        public static final double kCurrentLimit = 20;
        public static final double kRampRate = 0.2;

        public static final int ShooterIDLeftBottom = 90;
        public static final int ShooterIDLeftTop = 100;
        public static final int ShooterIDRightTop = 120;
        public static final int ShooterIDRightBottom = 110;

        // shooter motors config
        public static final TalonFXConfiguration rightShooterMotorsConfig = new TalonFXConfiguration() {
            {
                Slot0.kP = ShooterConstants.kP;
                Slot0.kI = ShooterConstants.kI;
                Slot0.kD = ShooterConstants.kD;
                Slot0.kS = ShooterConstants.kS;
                Slot0.kV = ShooterConstants.kV;
                Slot0.kA = ShooterConstants.kA;
                CurrentLimits.SupplyCurrentLimitEnable = true;
                CurrentLimits.SupplyCurrentLimit = kCurrentLimit;
                MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                MotorOutput.NeutralMode = NeutralModeValue.Coast;
                ClosedLoopRamps.VoltageClosedLoopRampPeriod=kRampRate;
            }
        };

        public static final TalonFXConfiguration leftShooterMotorsConfig = new TalonFXConfiguration() {
            {
                Slot0.kP = ShooterConstants.kP;
                Slot0.kI = ShooterConstants.kI;
                Slot0.kD = ShooterConstants.kD;
                Slot0.kS = ShooterConstants.kS;
                Slot0.kV = ShooterConstants.kV;
                Slot0.kA = ShooterConstants.kA;
                CurrentLimits.SupplyCurrentLimitEnable = true;
                CurrentLimits.SupplyCurrentLimit = kCurrentLimit;
                MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                MotorOutput.NeutralMode = NeutralModeValue.Coast;
                ClosedLoopRamps.VoltageClosedLoopRampPeriod=kRampRate;
            }
        };

        public static final AngularVelocity shooterAngularVelocity = RPM.of(2000);
        public static final AngularVelocity lobAngularVelocity = RPM.of(3200);
        public static final AngularVelocity restingAngularVelocity = RPM.of(0);
        public static final AngularVelocity hubAngularVelocity = RPM.of(2500);
        public static final AngularVelocity towerAngularVelocity = RPM.of(3000);

        public static final Angle hubHoodPitch = Degrees.of(75);
        public static final Angle towerHoodPitch = Degrees.of(70);

        public static final double backwardsTime = 0.05;

        // Old equation stuff
        // TODO: find ball launch velocity
        /** units: m/s */
        public static final double launchVelocity = Units.feetToMeters(29);// convert from ft/s to m/s
        public static final double ratio = 1;
        public static final double shootHeight = Units.inchesToMeters(40);

        public static AngularVelocity kShooterVelocityTolerance = RPM.of(500);
        public static Angle maxPitch = Degrees.of(85);

        public static final Distance kShooterWheelRadius = Inches.of(2);
        /** TODO: find the correct distance */
        public static final Distance kShooterRollerRadius = Inches.of(0.75);

        public static final double kShooterEfficiency = 0.85;

        // robot relative shooter offset
        // TODO implement in the calculation
        public static final Transform2d shooterOffset = new Transform2d(0, 0, new Rotation2d());

        public static enum AutoAimStatus {
            SHOOT(LEDConstants.LEDStatus.GREEN_BLINK),
            OUTOFRANGE(LEDConstants.LEDStatus.RED_BLINK),
            WAITING(LEDConstants.LEDStatus.YELLOW_BLINK);

            public LEDConstants.LEDStatus ledStatus;

            private AutoAimStatus(LEDConstants.LEDStatus ledStatus) {
                this.ledStatus = ledStatus;
            }
        }
    }

    public static class PhotonVisionConstants {

        // TODO: tune stddev values
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 4);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public final static Transform3d leftShooterCamTransform = new Transform3d(
                Units.inchesToMeters(-1.2887),
                Units.inchesToMeters(8.8466),
                Units.inchesToMeters(21.1190),
                new Rotation3d(
                        0,
                        Units.degreesToRadians(-30),
                        Units.degreesToRadians(-5)));
        public final static Transform3d rightShooterCamTransform = new Transform3d(
                Units.inchesToMeters(-1.2887),
                Units.inchesToMeters(-8.8466),
                Units.inchesToMeters(21.1190),
                new Rotation3d(
                        0,
                        Units.degreesToRadians(-30),
                        Units.degreesToRadians(5)));
    }

    public static class FieldConstants {
        public static final Map<DriverStation.Alliance, Pose2d> hubLocations = Map.of(
                Alliance.Blue, new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), new Rotation2d()),
                Alliance.Red,
                new Pose2d(Units.inchesToMeters(651.22 - 182.11), Units.inchesToMeters(158.84), new Rotation2d()));

        public static final Set<Pose2d> blueLobPositions = Set.of(
                new Pose2d(2.5, 2.1, new Rotation2d()),
                new Pose2d(2.5, 5.6, new Rotation2d()));

        public static final Set<Pose2d> redLobPositions = Set.of(
                new Pose2d(14.1, 2.1, new Rotation2d()),
                new Pose2d(14.1, 5.6, new Rotation2d()));

        public static final Map<DriverStation.Alliance, Set<Pose2d>> lobLocations = Map.of(
                Alliance.Blue,
                blueLobPositions,
                Alliance.Red,
                redLobPositions);

        public static enum TrenchZone {
            BLUE_LEFT(Inches.of(182.11), Inches.of(317.69 - 24.97)),
            BLUE_RIGHT(Inches.of(182.11), Inches.of(24.97)),
            RED_LEFT(Inches.of(651.22 - 182.11), Inches.of(24.97)),
            RED_RIGHT(Inches.of(651.22 - 182.11), Inches.of(317.69 - 24.97));

            public final Distance x, y;

            private TrenchZone(Distance x, Distance y) {
                this.x = x;
                this.y = y;
            }
        }

        public static Distance kFieldWidth = Inches.of(651.22);
        public static Distance kFieldHeight = Inches.of(317.69);

        public static Distance kAllianceWidth = Inches.of(156.61);
        public static Distance kAllianceHeight = kFieldHeight;

        public static Distance kTrenchX = Inches.of(182.11);
        public static Distance kTrenchZoneWidth = Inches.of(140.0);
        public static Distance kTrenchZoneHeight = Inches.of(49.96);

        public static Distance kTowerX = Inches.of(41.755);
        public static Distance kTowerY = Inches.of(147.47);
        public static Distance kTowerWidth = Inches.of(35.2);

        public static Distance kTrenchZoneYBuffer = DrivetrainConstants.kDrivetrainSizeY.div(4);

        public static enum FieldZone {
            FIELD(
                    new Translation2d(kFieldWidth.div(2), kFieldHeight.div(2)),
                    new Translation2d(kFieldWidth, kFieldHeight)),
            TRENCH_BLUE_LEFT(
                    new Translation2d(kTrenchX,
                            kFieldHeight.minus(kTrenchZoneHeight.div(2)).minus(kTrenchZoneYBuffer.div(2))),
                    new Translation2d(kTrenchX, kFieldHeight.minus(kTrenchZoneHeight.div(2))),
                    new Translation2d(kTrenchZoneWidth, kTrenchZoneHeight.plus(kTrenchZoneYBuffer))),
            TRENCH_BLUE_RIGHT(
                    new Translation2d(kTrenchX, kTrenchZoneHeight.div(2).plus(kTrenchZoneYBuffer.div(2))),
                    new Translation2d(kTrenchX, kTrenchZoneHeight.div(2)),
                    new Translation2d(kTrenchZoneWidth, kTrenchZoneHeight.plus(kTrenchZoneYBuffer))),
            TRENCH_RED_LEFT(
                    new Translation2d(kFieldWidth.minus(kTrenchX),
                            kTrenchZoneHeight.div(2).plus(kTrenchZoneYBuffer.div(2))),
                    new Translation2d(kFieldWidth.minus(kTrenchX), kTrenchZoneHeight.div(2)),
                    new Translation2d(kTrenchZoneWidth, kTrenchZoneHeight.plus(kTrenchZoneYBuffer))),
            TRENCH_RED_RIGHT(
                    new Translation2d(kFieldWidth.minus(kTrenchX),
                            kFieldHeight.minus(kTrenchZoneHeight.div(2)).minus(kTrenchZoneYBuffer.div(2))),
                    new Translation2d(kFieldWidth.minus(kTrenchX), kFieldHeight.minus(kTrenchZoneHeight.div(2))),
                    new Translation2d(kTrenchZoneWidth, kTrenchZoneHeight.plus(kTrenchZoneYBuffer)));

            public final Translation2d center, focus, size;

            private FieldZone(Translation2d center, Translation2d focus, Translation2d size) {
                this.center = center;
                this.focus = focus;
                this.size = size;
            }

            private FieldZone(Translation2d center, Translation2d size) {
                this(center, center, size);
            }
        }

        public static final FieldZone[] kTrenchZones = new FieldZone[] {
                FieldZone.TRENCH_BLUE_LEFT,
                FieldZone.TRENCH_BLUE_RIGHT,
                FieldZone.TRENCH_RED_LEFT,
                FieldZone.TRENCH_RED_RIGHT
        };

        /** Units:m/s^2 */
        public static final double g = 9.80;

        public static final double hubZ = Units.inchesToMeters(56.4);
    }

    public static class AutoConstants {
        /** units: seconds */
        public static final double ShootTime = 3;

        public static final APConstraints kAutopilotConstraints = new APConstraints()
                .withAcceleration(7.0)
                .withJerk(3.5);
        public static final APProfile kAutopilotAlignProfile = new APProfile(kAutopilotConstraints)
                .withErrorXY(Centimeters.of(2.0))
                .withErrorTheta(Degrees.of(3600.0))
                .withBeelineRadius(Centimeters.of(5.0));
        public static final Autopilot kAutopilotAlignController = new Autopilot(kAutopilotAlignProfile);
        public static final APProfile kAutopilotCruiseProfile = new APProfile(kAutopilotConstraints)
                .withErrorXY(Centimeters.of(20.0))
                .withErrorTheta(Degrees.of(3600.0))
                .withBeelineRadius(Centimeters.of(25.0));
        public static final Autopilot kAutopilotCruiseController = new Autopilot(kAutopilotCruiseProfile);
    }

    public static class AlignConstants {
        // TODO: tune acceleration and jerk for aligning
        public static final APConstraints kAlignConstraints = new APConstraints()
                .withVelocity(kMaxTranslationSpeed)
                .withAcceleration(6.0)
                .withJerk(3.0);
        public static final APProfile kAlignProfile = new APProfile(kAlignConstraints)
                .withErrorXY(Centimeters.of(2.0))
                .withErrorTheta(Degrees.of(1.0))
                .withBeelineRadius(Centimeters.of(5.0));
        public static final Autopilot kAlignController = new Autopilot(kAlignProfile);

        public static enum TowerAlignGoal {
            LEFT(
                    new APTarget(new Pose2d(FieldConstants.kTowerX,
                            FieldConstants.kTowerY.plus(FieldConstants.kTowerWidth.div(2))
                                    .plus(DrivetrainConstants.kDrivetrainSizeX.div(2)),
                            Rotation2d.kZero)).withVelocity(0).withoutEntryAngle()),
            RIGHT(
                    new APTarget(new Pose2d(FieldConstants.kTowerX,
                            FieldConstants.kTowerY.minus(FieldConstants.kTowerWidth.div(2))
                                    .minus(DrivetrainConstants.kDrivetrainSizeX.div(2)),
                            Rotation2d.k180deg)).withVelocity(0).withEntryAngle(Rotation2d.k180deg)),
            MIDDLE_FRONT(
                    new APTarget(new Pose2d(
                            FieldConstants.kTowerX.plus(DrivetrainConstants.kDrivetrainSizeX.div(2)).plus(Inches.of(6)),
                            FieldConstants.kTowerY, Rotation2d.k180deg)).withVelocity(0).withoutEntryAngle(),
                    new APTarget(new Pose2d(FieldConstants.kTowerX, FieldConstants.kTowerY, Rotation2d.k180deg))
                            .withVelocity(0).withoutEntryAngle()),
            MIDDLE_BACK(
                    new APTarget(new Pose2d(
                            FieldConstants.kTowerX.minus(DrivetrainConstants.kDrivetrainSizeX.div(2))
                                    .minus(Inches.of(6)),
                            FieldConstants.kTowerY, Rotation2d.kZero)).withVelocity(0).withoutEntryAngle(),
                    new APTarget(new Pose2d(FieldConstants.kTowerX, FieldConstants.kTowerY, Rotation2d.kZero))
                            .withVelocity(0).withoutEntryAngle());

            public final APTarget[] targets;

            private TowerAlignGoal(APTarget... targets) {
                this.targets = targets;
            }
        }
    }

    public static final class LEDConstants {

        private static final Distance ledSpacing = Meters.of(1.0 / 120.0);

        public static enum LEDStatus {
            RAINBOW(LEDPattern.rainbow(255, 64)),
            RAINBOW_SCROLL(RAINBOW.pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing)),

            RED_SOLID(LEDPattern.solid(Color.kRed)),
            YELLOW_SOLID(LEDPattern.solid(Color.kYellow)),
            BLUE_SOLID(LEDPattern.solid(Color.kBlue)),
            GREEN_SOLID(LEDPattern.solid(Color.kGreen)),

            RED_BLINK(RED_SOLID.getPattern().blink(Seconds.of(0.25), Seconds.of(0.25))),
            YELLOW_BLINK(YELLOW_SOLID.getPattern().blink(Seconds.of(0.25), Seconds.of(0.25))),
            GREEN_BLINK(GREEN_SOLID.getPattern().blink(Seconds.of(0.25), Seconds.of(0.25)));

            private LEDPattern pattern;

            LEDStatus(LEDPattern c_InPattern) {
                this.pattern = c_InPattern;
            }

            public LEDPattern getPattern() {
                return pattern;
            }
        }
    }

    public static final class JoeLookupTableConstants {

      public static record LookupTablePoint(AngularVelocity angularVelocity, double efficiency) {};

        /**
         * adjust the angle of the hood down by this much (in radians for each
         * meter/second slow the calculated tangential speed is
         * multiplied by the distance from the hub (higher distance needs more
         * correction)
         */
        public static final double kShooterDistanceVelocityCorrection = 0.01;

        public static final Distance kMaxDistance = Meters.of(4.5);

        // stores desired velocity based on position
        public static final Map<Distance, LookupTablePoint> joeLookupTable = Map.ofEntries(
                Map.entry(Meters.of(1),   new LookupTablePoint(RPM.of(2600), 0.82)),
                Map.entry(Meters.of(1.5), new LookupTablePoint(RPM.of(2600), 0.81)),
                Map.entry(Meters.of(2),   new LookupTablePoint(RPM.of(2600), 0.80)),
                Map.entry(Meters.of(2.5), new LookupTablePoint(RPM.of(2700), 0.79)),
                Map.entry(Meters.of(3),   new LookupTablePoint(RPM.of(2800), 0.78)),
                Map.entry(Meters.of(3.5), new LookupTablePoint(RPM.of(2900), 0.77)),
                Map.entry(Meters.of(4),   new LookupTablePoint(RPM.of(3100), 0.76)),
                Map.entry(Meters.of(4.5), new LookupTablePoint(RPM.of(3300), 0.75))
        );
    }
}
