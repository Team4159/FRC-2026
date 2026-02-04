// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

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
  }
}
