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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
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
      CLIMB(0.25),STOP(0),DOWN(-0.25);
      public double percentage;
      private ClimberState(double speed) {
        percentage = speed;
      }
    }
  }

  public static class HopperConstants {
    //TODO: set later
    public static final int HopperId = 30;

    public static enum HopperState {
      FEED(-1),REVERSE(1),STOP(0);
      public double percentage;
      private HopperState(double speed){
        percentage = speed;
      }
    }
  }
  public static class FeederConstants{
    public static final int FeederID = 20; //idk if this port is used yet plz check

    public static enum FeederState{
      FEED(0.75), UNSTUCKFEEDER (-0.75), STOP(0);
      public double percentage;
      private FeederState(double speed){
        percentage = speed;
      }
    }
  }

  public static class IntakeConstants{
    public static final double kAngleI = 0.1;
    public static final double kAngleD = 0;
    public static final double kAngleP = 20;
    public static final double kAngleG = 0.08;

    //motion magic
    public static final double kFastCruiseVelocity = 160;
    public static final double kFastAcceleration = 500;
    public static final double kFastJerk = 1600;

    public static final double kSlowCruiseVelocity = 1;
    public static final double kSlowAcceleration = 5;
    public static final double kSlowJerk = 1600;

    public static final int kAngleEncoderId = 1;
    public static final int kAngleId = 6; //youre welcome Faye
    public static final int kIntakeSpinId = 7;

    public static final Angle kEncoderOffset = Degrees.of(80);
    public static final double kMotorToSensorRatio = 50;
    public static final double kSensorToMechanismRatio = 1;

    //motor configs
    public static final CANcoderConfiguration canCoderConfig = new CANcoderConfiguration(){{
      MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
      MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
      MagnetSensor.withMagnetOffset(IntakeConstants.kEncoderOffset);
    }};

    public static final TalonFXConfiguration angleConfig = new TalonFXConfiguration(){{
      Slot0.kP = IntakeConstants.kAngleP;  
      Slot0.kI = IntakeConstants.kAngleI;
      Slot0.kD = IntakeConstants.kAngleD;
      Slot0.kG = IntakeConstants.kAngleG;
      Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
      //abs encoder
      Feedback.FeedbackRemoteSensorID = IntakeConstants.kAngleEncoderId;
      Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      Feedback.SensorToMechanismRatio = IntakeConstants.kSensorToMechanismRatio;
      Feedback.RotorToSensorRatio = IntakeConstants.kMotorToSensorRatio;
    }};

    //motion magic
    public static final MotionMagicConfigs kFastMotionMagicConfig = new MotionMagicConfigs(){{
      MotionMagicCruiseVelocity = IntakeConstants.kFastCruiseVelocity; // Target cruise velocity of 80 rps
      MotionMagicAcceleration = IntakeConstants.kFastAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
      MotionMagicJerk = IntakeConstants.kFastJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)
    }};

    public static final MotionMagicConfigs kSlowMotionMagicConfig = new MotionMagicConfigs(){{
      MotionMagicCruiseVelocity = IntakeConstants.kSlowCruiseVelocity;
      MotionMagicAcceleration = IntakeConstants.kSlowAcceleration;
      MotionMagicJerk = IntakeConstants.kSlowJerk;
    }};
    
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
      DOWN_ON(Degrees.of(-12), 0.75), 
      DOWN_OFF(Degrees.of(-12), 0), 
      DOWN_REV(Degrees.of(-12), -0.75),
      UP_OFF(Degrees.of(130), 0), 
      BOUNCE_UP(Degrees.of(45), 0),
      STOP(Degrees.of(130), 0); 

      public final Angle rotationLocation;
      public final double spinSpeed;

      private IntakeState(Angle location, double speed){
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
            FREE,
            MANUAL_ALIGN,
            BRAKE,
            POINT,
            IDLE,
            INTAKE_FORWARD,
            INTAKE_LEFT,
            INTAKE_RIGHT,
            RADIAL,
        }

        // drive assist constants
        public static final Distance kTrenchAssistPassPositionTolerance = Meters.of(0.45);
        public static final double kTrenchAssistApproachInputTolerance = 0.2;
        public static final Distance kTrenchAssistAlignPositionTolerance = Meters.of(0.15);
        public static final double kTrenchAssistAlignStrength = 0.8;
        public static final double kTrenchAssistAlignInfluence = 0.15;

        // drive mode constants
        public static final Angle kPrimaryAutoBrakeReachedDesiredAngleTolerance = Degrees.of(5);

        public static final double kPrimaryReduceSpeedTranslationFactor = 0.25;
        public static final double kPrimaryReduceSpeedRotationFactor = 1;

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

        public static final double kAimKP = 5.0;
        public static final double kAimKI = 0.0;
        public static final double kAimKD = 0.0;
        public static final double kAimFeedForward = 0.0;

        public static final Angle AutoAimTolerance = Degrees.of(5);

        public static final PhoenixPIDController AutoAimRotationController = new PhoenixPIDController(kAimKP, kAimKI, kAimKD);
        static {
            AutoAimRotationController.enableContinuousInput(-Math.PI, Math.PI);
        }

    }

  public static class ShooterConstants {

    public static final double kHoodI = 1;
    public static final double kHoodD = 0;
    public static final double kHoodP = 100;
    public static final double kHoodG = 0.25;
    public static final double kHoodS = 0.1;
    // public static final double kHoodV = 1.11;
    // public static final double kHoodA = 0.17;
    //abs encoder
    public static final int HoodId = 8;
    public static final int kHoodEncoderID = 2;
    public static final Angle kEncoderOffset = Degrees.of(-235);
    public static final double kSensorToMechanismRatio = 34/16;
    public static final double kMotorToSensorRatio = 125;

    public static final double kCruiseVelocity = 40;
    public static final double kAcceleration = 80;
    public static final double kJerk = 1600;

    //hood cancoder
    public static final CANcoderConfiguration canCoderConfig = new CANcoderConfiguration(){{
      MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
      MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
      MagnetSensor.withMagnetOffset(ShooterConstants.kEncoderOffset);
    }};

    //motion magic
    public static final MotionMagicConfigs kHoodMotionMagicConfig = new MotionMagicConfigs(){{
      MotionMagicCruiseVelocity = ShooterConstants.kCruiseVelocity; // Target cruise velocity of 80 rps
      MotionMagicAcceleration = ShooterConstants.kAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
      MotionMagicJerk = ShooterConstants.kJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)
    }};
    //hood motor conifg
    public static final TalonFXConfiguration hoodConfig = new TalonFXConfiguration(){{
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
      //abs encoder
      Feedback.FeedbackRemoteSensorID = ShooterConstants.kHoodEncoderID;
      Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      Feedback.SensorToMechanismRatio = ShooterConstants.kSensorToMechanismRatio;
      Feedback.RotorToSensorRatio = ShooterConstants.kMotorToSensorRatio;
      MotionMagic = kHoodMotionMagicConfig;
    }};

    //Shooter Motor Config and PID
    public static final double kP = 100;
    public static final double kI = 5;
    public static final double kD = 0;
    public static final int ShooterIDLeftBottom = 9;
    public static final int ShooterIDLeftTop = 10;
    public static final int ShooterIDRightTop = 11;
    public static final int ShooterIDRightBottom = 12;

    //shooter motors config
    public static final TalonFXConfiguration rightShooterMotorsConfig = new TalonFXConfiguration(){{
      Slot0.kP = ShooterConstants.kP;
      Slot0.kI = ShooterConstants.kI;
      Slot0.kD = ShooterConstants.kD;
      CurrentLimits.SupplyCurrentLimitEnable = true;
      CurrentLimits.SupplyCurrentLimit = 40;
      MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }};

    public static final TalonFXConfiguration leftShooterMotorsConfig = new TalonFXConfiguration(){{
      Slot0.kP = ShooterConstants.kP;
      Slot0.kI = ShooterConstants.kI;
      Slot0.kD = ShooterConstants.kD;
      CurrentLimits.SupplyCurrentLimitEnable = true;
      CurrentLimits.SupplyCurrentLimit = 40;
      MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }};

    public static final AngularVelocity shooterAngularVelocity = RPM.of(3000);
    public static final AngularVelocity lobAngularVelocity = RPM.of(3200);
    public static final AngularVelocity restingAngularVelocity = RPM.of(1500);


    //Old equation stuff
    //TODO: find ball launch velocity
    /** units: m/s */
    public static final double launchVelocity = Units.feetToMeters(29);//convert from ft/s to m/s
    public static final double ratio = 1;
    public static final double shootHeight = Units.inchesToMeters(40);

    public static AngularVelocity kShooterVelocityTolerance = RPM.of(100);

    public static final Distance kShooterWheelRadius = Inches.of(2);
    /** TODO: find the correct distance */
    public static final Distance kShooterRollerRadius = Inches.of(0.75);

    public static final double kShooterEfficiency = 0.6;

    //robot relative shooter offset
    //TODO implement in the calculation
    public static final Transform2d shooterOffset = new Transform2d(0, 0, new Rotation2d());

    public static enum AutoAimStatus{
      SHOOT(LEDConstants.LEDStatus.GREEN_BLINK),
      OUTOFRANGE(LEDConstants.LEDStatus.RED_BLINK),
      WAITING(LEDConstants.LEDStatus.YELLOW_BLINK);

      public LEDConstants.LEDStatus ledStatus;
      private AutoAimStatus(LEDConstants.LEDStatus ledStatus){
        this.ledStatus = ledStatus;
      }
    }
  }

  public static class PhotonVisionConstants {

        // TODO: tune stddev values
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
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

        public static final Set<Pose2d> blueLobPositions = 
          Set.of(new Pose2d(2.5, 2.1, new Rotation2d()),
          new Pose2d(2.5, 5.6, new Rotation2d()));

        public static final Set<Pose2d> redLobPositions = 
          Set.of(new Pose2d(14.1, 2.1, new Rotation2d()),
          new Pose2d(14.1, 5.6, new Rotation2d()));

        public static final Map<DriverStation.Alliance, Set<Pose2d>> lobLocations = Map.of(
                Alliance.Blue, 
                blueLobPositions,
                Alliance.Red,
                redLobPositions);

        public static enum FieldZone {
            FIELD(Inches.of(651.22), Inches.of(317.69)),
            ALLIANCE(Inches.of(156.61), Inches.of(317.69)),
            TRENCH(Inches.of(140.0), Inches.of(49.96));

            public final Distance width, height;

            private FieldZone(Distance width, Distance height) {
                this.width = width;
                this.height = height;
            }
        }

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

        public static Distance kTrenchX = Inches.of(182.11);

        public static Distance kTowerX = Inches.of(41.755);
        public static Distance kTowerY = Inches.of(147.47);
        public static Distance kTowerWidth = Inches.of(35.2);

        /** Units:m/s^2 */
        public static final double g = 9.80;

        public static final double hubZ = Units.inchesToMeters(56.4);
    }

    public static class AutoConstants {
      /** units: seconds */
      public static final double ShootTime = 3;
    }

    public static class AlignConstants {
        // TODO: tune acceleration and jerk for aligning
        public static final APConstraints kAlignConstraints = new APConstraints()
                .withVelocity(kMaxTranslationSpeed)
                .withAcceleration(7.0)
                .withJerk(3.5);
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
                    new APTarget(new Pose2d(FieldConstants.kTowerX.plus(DrivetrainConstants.kDrivetrainSizeX.div(2)).plus(Inches.of(6)),
                            FieldConstants.kTowerY, Rotation2d.k180deg)).withVelocity(0).withoutEntryAngle(),
                    new APTarget(new Pose2d(FieldConstants.kTowerX, FieldConstants.kTowerY, Rotation2d.k180deg)).withVelocity(0).withoutEntryAngle()),
            MIDDLE_BACK(
                    new APTarget(new Pose2d(FieldConstants.kTowerX.minus(DrivetrainConstants.kDrivetrainSizeX.div(2)).minus(Inches.of(6)),
                            FieldConstants.kTowerY, Rotation2d.kZero)).withVelocity(0).withoutEntryAngle(),
                    new APTarget(new Pose2d(FieldConstants.kTowerX, FieldConstants.kTowerY, Rotation2d.kZero)).withVelocity(0).withoutEntryAngle());

            public final APTarget[] targets;

            private TowerAlignGoal(APTarget... targets) {
                this.targets = targets;
            }
        }
    }

  public static final class LEDConstants{
    
    private static final Distance ledSpacing = Meters.of(1.0 / 120.0);

    public static enum LEDStatus { 
        RAINBOW(LEDPattern.rainbow(255,64)),
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
  public static final class JoeLookupTableConstants{

    public static final record ShotData(Angle angle, Time time){
      public double getAngleRadians(){return angle.in(Radians);}
      public double getTimeSeconds(){return time.in(Seconds);}
    }

    /**adjust the angle of the hood down by this much (in radians) for each meter/second slow the calculated tangential speed is*/
    public static final double kShooterVelocityCorrection = 0.05;
    /**adjust the angle of the hood down by this much (in radians for each meter/second slow the calculated tangential speed is
     * multiplied by the distance from the hub (higher distance needs more correction)
    */
    public static final double kShooterDistanceVelocityCorrection = 0.01;

    public static final Distance kMaxDistance = Meters.of(4.5);

    //stores desired angle and estimated time (from stationary) given a distance from the hub
    public static final Map<Distance, ShotData> joeLookupTable = Map.ofEntries(
      Map.entry(Meters.of(0),   new ShotData(Degrees.of(87), Seconds.of(1.7277))),
      Map.entry(Meters.of(0.5), new ShotData(Degrees.of(84), Seconds.of(1.7277))),
      Map.entry(Meters.of(1),   new ShotData(Degrees.of(82), Seconds.of(1.7277))),
      Map.entry(Meters.of(1.5), new ShotData(Degrees.of(81), Seconds.of(1.726))),
      Map.entry(Meters.of(2),   new ShotData(Degrees.of(80), Seconds.of(1.718))),
      Map.entry(Meters.of(2.5), new ShotData(Degrees.of(78), Seconds.of(1.708))),
      Map.entry(Meters.of(3),   new ShotData(Degrees.of(76), Seconds.of(1.697))),
      Map.entry(Meters.of(3.5), new ShotData(Degrees.of(74), Seconds.of(1.682))),
      Map.entry(Meters.of(4),   new ShotData(Degrees.of(72), Seconds.of(1.664)))
      // Map.entry(Meters.of(4.5), new ShotData(Degrees.of(66.726), Seconds.of(1.642))),
      //Map.entry(Meters.of(5.5), new ShotData(Degrees.of(47), Seconds.of(1.556))),
      //Map.entry(Meters.of(6),   new ShotData(Degrees.of(45), Seconds.of(1.512)))
      // Map.entry(Meters.of(6.5), new ShotData(Degrees.of(53.808), Seconds.of(1.451))),
      // Map.entry(Meters.of(7),   new ShotData(Degrees.of(50.122), Seconds.of(1.349))),
      // Map.entry(Meters.of(7.5), new ShotData(Degrees.of(45),     Seconds.of(1.176))),
      // Map.entry(Meters.of(8),   new ShotData(Degrees.of(45),     Seconds.of(1.176))),
      // Map.entry(Meters.of(8.5), new ShotData(Degrees.of(45),     Seconds.of(1.176))),
      // Map.entry(Meters.of(9),   new ShotData(Degrees.of(45),     Seconds.of(1.176))),
      // Map.entry(Meters.of(9.5), new ShotData(Degrees.of(45),     Seconds.of(1.176)))
    );
  }
}
