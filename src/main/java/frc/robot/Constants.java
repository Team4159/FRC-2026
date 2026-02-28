// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DrivetrainConstants.kMaxTranslationSpeed;

import java.util.Map;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
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
    public static final int idClimberOne = 13;
    public static final int idClimberTwo = 14;

    public static enum ClimberState {
      CLIMB(1),STOP(0),DOWN(-1);
      public double percentage;
      private ClimberState(double speed) {
        percentage = speed;
      }
    }
  }

  public static class HopperConstants {
    public static final int HopperId = 15;

    public static enum HopperState {
      FEED(0.5),REVERSE(-0.5),STOP(0);
      public double percentage;
      private HopperState(double speed){
        percentage = speed;
      }
    }
  }
  public static class FeederConstants{
    public static final int FeederID = 20; //idk if this port is used yet plz check

    public static enum FeederState{
      FEED(0.5), UNSTUCKFEEDER (-0.5), STOP(0);
      public double percentage;
      private FeederState(double speed){
        percentage = speed;
      }
    }
  }

  public static class IntakeConstants{
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kP = 7;

    public static final int kIntakeLocationId = 6; //youre welcome Faye
    public static final int kIntakeSpinId = 7; 
    public static final int kCANCoderID = 1;

    //public static final double kLocationGearRatio = 1.0 / 2.0;
    //public static final double kSpinGearRatio = 1.0 / 5.0;

    public static enum IntakeState {
      DOWN_ON(0.46, -0.75), DOWN_OFF(0.46, 0), UP_OFF(0.05, 0), STOP(0.05, 0); 

      public final double rotationLocation;
      public final double spinSpeed;

      private IntakeState(double location, double speed){
        rotationLocation = location;
        spinSpeed = speed;
      }
    }
  }

    public static class OperatorConstants {
        public static final int kPrimaryControllerPort = 0;

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
            INTAKE,
            RADIAL,
        }

        // drive assist constants
        public static final Distance kTrenchAssistPassPositionTolerance = Meters.of(0.45);
        public static final double kTrenchAssistApproachInputTolerance = 0.2;
        public static final Distance kTrenchAssistAlignPositionTolerance = Meters.of(0.15);
        public static final double kTrenchAssistAlignStrength = 0.8;
        public static final double kTrenchAssistAlignInfluence = 0.15;

        // drive mode constants
        public static final double kPrimaryReduceSpeedTranslationFactor = 0.5;
        public static final double kPrimaryReduceSpeedRotationFactor = 1;

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

        public static final int kPigeonId = 1;

        public static final PhoenixPIDController AutoAimRotationController = new PhoenixPIDController(5, 0, 0);
        static {
            AutoAimRotationController.enableContinuousInput(-Math.PI, Math.PI);
        }
    }

  public static class ShooterConstants {
    //Motor Config and PID
    
    public static final double kIHood = 0.05;
    public static final double kDHood = 0.01;
    public static final double kPHood = 40;
    public static final double kFeedForward = 1;

    public static final double kPShooter = 2;
    public static final double kIShooter = 0;
    public static final double kDShooter = 0;

    public static final int ShooterIDOne = 9;
    public static final int ShooterIDTwo = 10;
    public static final int ShooterIDThree = 11;
    public static final int ShooterIDFour = 12;
    
    public static final int hoodID = 8;
    public static final int kHoodCANCoderID = 2;

    public static final double kHoodRatio = 3;
    public static final Angle angleOffset = Degrees.of(0);


    //TODO: find ball launch velocity
    /** units: m/s */
    public static final double launchVelocity = Units.feetToMeters(29);//convert from ft/s to m/s
    public static final double ratio = 1;
    public static final double shootHeight = Units.inchesToMeters(40);

    public static final Distance kShooterWheelRadius = Inches.of(2);
    /** TODO: find the correct distance */
    public static final Distance kShooterRollerRadius = Inches.of(0.75);

    public static final double kShooterEfficiency = 0.9;

    /** units: radians */
    public static final double maxPitch = Units.degreesToRadians(85);

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
                Units.inchesToMeters(-0.1647),
                Units.inchesToMeters(8.8160),
                Units.inchesToMeters(20.3287),
                new Rotation3d(
                        0,
                        Units.degreesToRadians(-30),
                        Units.degreesToRadians(-5)));
        public final static Transform3d rightShooterCamTransform = new Transform3d(
                Units.inchesToMeters(-0.1647),
                Units.inchesToMeters(-8.8160),
                Units.inchesToMeters(20.3287),
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
            BLUE_LEFT(Inches.of(182.11), Inches.of(24.97)),
            BLUE_RIGHT(Inches.of(182.11), Inches.of(317.69 - 24.97)),
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
        public static final APConstraints kAlignConstraints = new APConstraints()
                .withVelocity(kMaxTranslationSpeed)
                .withAcceleration(5.0)
                .withJerk(2.0);
        public static final APProfile kAlignProfile = new APProfile(kAlignConstraints)
                .withErrorXY(Centimeters.of(2.0))
                .withErrorTheta(Degrees.of(1.0))
                .withBeelineRadius(Centimeters.of(5.0));
        public static final Autopilot kAlignController = new Autopilot(kAlignProfile);

        public static enum TowerAlignGoal {
            // TODO: decide if LEFT and RIGHT should have an alignment setpoint or just an entry angle
            LEFT(
                    new APTarget(new Pose2d(FieldConstants.kTowerX,
                            FieldConstants.kTowerY.plus(FieldConstants.kTowerWidth.div(2))
                                    .plus(DrivetrainConstants.kDrivetrainSizeX.div(2)),
                            Rotation2d.kZero)).withVelocity(0).withEntryAngle(Rotation2d.kZero)),
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
}
