// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerVelocityDeadband = 0.1;
    public static final double kDriveControllerRotationDeadband = 0.1;
  }

  public static class DrivetrainConstants {
    public static final PhoenixPIDController AutoAimRotationController = new PhoenixPIDController(15, 0, 0);
    static {
      AutoAimRotationController.enableContinuousInput(-Math.PI, Math.PI);
    }
  }

  public static class ShooterConstants {
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
