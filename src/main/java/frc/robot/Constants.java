// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LEDConstants.LEDStatus;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class HoodConstants {
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kP = 0.01;
    public static final int HoodId = 3; //fix this
  }

  public static class ClimberConstants {
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kP = 0;
    public static final int idClimberOne = 9;
    public static final int idClimberTwo = 10;

    public static enum ClimberState {
      CLIMB(1),STOP(0),DOWN(-1);
      public double percentage;
      private ClimberState(double speed) {
        percentage = speed;
      }
    }
  }

  public static class HopperConstants {
    public static final int HopperId = 8;

    public static enum HopperState {
      FEED(0.5),REVERSE(-0.5),STOP(0);
      public double percentage;
      private HopperState(double speed){
        percentage = speed;
      }
    }
  }
  public static class FeederConstants{
    public static final int FeederID = 9; //idk if this port is used yet plz check

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
    public static final double kP = 0.01;

    public static final int kIntakeLocationId = 9; //I don't know the port, change once known.
    public static final int kIntakeSpinId = 10;

    public static final double kLocationGearRatio = 1.0 / 2.0;
    public static final double kSpinGearRatio = 1.0 / 5.0;

    public static enum IntakeState {
      DOWN_ON(0.25, 0.5), DOWN_OFF(0.25, 0), UP_OFF(0, 0), STOP(0, 0); 

      public final double rotationLocation;
      public final double spinSpeed;

      private IntakeState(double location, double speed){
        rotationLocation = location;
        spinSpeed = speed;
      }
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerTranslationDeadband = 0.2;
    public static final double kDriverControllerRotationDeadband = 0.2;
  }

  public static class DrivetrainConstants {
    public static final PhoenixPIDController AutoAimRotationController = new PhoenixPIDController(15, 0, 0);
    static {
      AutoAimRotationController.enableContinuousInput(-Math.PI, Math.PI);
    }
  }

  public static class ShooterConstants {
    //Motor Config and PID
    public static final double kP = 0.01;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int ShooterIDOne = 4;
    public static final int ShooterIDTwo = 5;
    public static final int ShooterIDThree = 6;
    public static final int ShooterIDFour = 7;


    //TODO: find ball launch velocity
    /** units: m/s */
    public static final double launchVelocity = Units.feetToMeters(29);//convert from ft/s to m/s
    public static final double ratio = 1;
    public static final double shootHeight = Units.inchesToMeters(40);

    /** units: radians */
    public static final double maxPitch = Units.degreesToRadians(85);

    //robot relative shooter offset
    //TODO implement in the calculation
    public static final Transform2d shooterOffset = new Transform2d(0, 0, new Rotation2d());

    public static enum AutoAimStatus{
      SHOOT(LEDStatus.GREEN_BLINK),
      OUTOFRANGE(LEDStatus.RED_BLINK),
      WAITING(LEDStatus.YELLOW_BLINK);

      public LEDStatus ledStatus;
      private AutoAimStatus(LEDStatus ledStatus){
        this.ledStatus = ledStatus;
      }
    }
  }

  public static class PhotonVisionConstants {
    public static Transform3d leftShooterCamTransform = new Transform3d(-0.1647, 8.8160, 20.3287,new Rotation3d(0,-30,-5));
    public static Transform3d rightShooterCamTransform = new Transform3d(-0.1647, -8.8160, 20.8237, new Rotation3d(0, -30, 5));
  }

  public static class FieldConstants {
    public static final Map<DriverStation.Alliance, Pose2d> hubLocations = Map.of(
      Alliance.Blue, new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), new Rotation2d()),
      Alliance.Red, new Pose2d(Units.inchesToMeters(651.22 - 182.11), Units.inchesToMeters(158.84), new Rotation2d())
    );

    /** Units:m/s^2 */
    public static final double g = 9.80;

    public static final double hubZ = Units.inchesToMeters(56.4);
  }

  public static final class JoeLookupTableConstants{
    public static final record ShotData(Angle angle, Time time){
      public double getAngleRadians(){return angle.in(Radians);}
      public double getTimeSeconds(){return time.in(Seconds);}
    }
    //stores desired angle and estimated time (from stationary) given a distance from the hub
    public static final Map<Distance, ShotData> joeLookupTable = Map.ofEntries(
      Map.entry(Meters.of(0),   new ShotData(Degrees.of(85),     Seconds.of(1.7277))),
      Map.entry(Meters.of(0.5), new ShotData(Degrees.of(85),     Seconds.of(1.7277))),
      Map.entry(Meters.of(1),   new ShotData(Degrees.of(85),     Seconds.of(1.7277))),
      Map.entry(Meters.of(1.5), new ShotData(Degrees.of(84.292), Seconds.of(1.726))),
      Map.entry(Meters.of(2),   new ShotData(Degrees.of(82.347), Seconds.of(1.718))),
      Map.entry(Meters.of(2.5), new ShotData(Degrees.of(80.364), Seconds.of(1.708))),
      Map.entry(Meters.of(3),   new ShotData(Degrees.of(78.329), Seconds.of(1.697))),
      Map.entry(Meters.of(3.5), new ShotData(Degrees.of(76.226), Seconds.of(1.682))),
      Map.entry(Meters.of(4),   new ShotData(Degrees.of(74.034), Seconds.of(1.664))),
      Map.entry(Meters.of(4.5), new ShotData(Degrees.of(71.726), Seconds.of(1.642))),
      Map.entry(Meters.of(5),   new ShotData(Degrees.of(68.689), Seconds.of(1.591))),
      Map.entry(Meters.of(5.5), new ShotData(Degrees.of(65.879), Seconds.of(1.556))),
      Map.entry(Meters.of(6),   new ShotData(Degrees.of(62.686), Seconds.of(1.512))),
      Map.entry(Meters.of(6.5), new ShotData(Degrees.of(58.808), Seconds.of(1.451))),
      Map.entry(Meters.of(7),   new ShotData(Degrees.of(53.122), Seconds.of(1.349))),
      Map.entry(Meters.of(7.5), new ShotData(Degrees.of(45),     Seconds.of(1.176))),
      Map.entry(Meters.of(8),   new ShotData(Degrees.of(45),     Seconds.of(1.176))),
      Map.entry(Meters.of(8.5), new ShotData(Degrees.of(45),     Seconds.of(1.176))),
      Map.entry(Meters.of(9),   new ShotData(Degrees.of(45),     Seconds.of(1.176))),
      Map.entry(Meters.of(9.5), new ShotData(Degrees.of(45),     Seconds.of(1.176)))
    );
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
