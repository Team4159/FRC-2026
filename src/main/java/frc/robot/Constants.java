// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class ShooterConstants {
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kP = 0.01;
    public static final int ShooterIDOne = 4;
    public static final int ShooterIDTwo = 5;
    public static final int ShooterIDThree = 6;
    public static final int ShooterIDFour = 7;
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

  public static class IntakeConstants{
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kP = 0.01;

    public static final int kIntakeLocationId = 9; //I don't know the port, change once known.
    public static final int kIntakeSpinId = 10;

    public static enum IntakeLocationState {
      UP(0), DOWN(0.25); //I don't know the gear ratio

      public double rotationLocation;

      private IntakeLocationState(double location){
        rotationLocation = location;
      }
    }

    public static enum IntakeSpinState {
      IN(0.5), OUT(-0.5), STOP(0);

      public double spinSpeed;

      private IntakeSpinState(double speed) {
        spinSpeed = speed;
      }
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
