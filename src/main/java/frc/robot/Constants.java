// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;

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
      public static final int kPrimaryControllerPort = 0;
      public static final double kPrimaryTranslationDeadband = 0.1;
      public static final double kPrimaryRotationDeadband = 0.1;
      public static final double kPrimaryTranslationExponent = 2.0;
      public static final double kPrimaryRotationExponent = 2.0;
      public static final double kPrimaryTranslationRadius = 0.95;
      public static final double kPrimaryRotationRadius = 0.95;
      public static final double kPrimaryRadialModeDeadband = 0.2;

      public static enum DriveMode {
          FIELD_CENTRIC,
          ROBOT_CENTRIC,
          BRAKE,
          POINT,
          IDLE,
          INTAKE,
          SHOOT,
      }
  }

  public static class DrivetrainConstants {
    public static final double kMaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // driver assist constnats
    public static Distance kTrenchAssistPassPositionTolerance = Meters.of(0.3);
    public static double kTrenchAssistPassInputTolerance = 0.25;
    public static Distance kTrenchAssistAlignPositionTolerance = Meters.of(0.15);
    public static double kTrenchAssistAlignInfluence = 0.15;

    public static final PhoenixPIDController AutoAimRotationController = new PhoenixPIDController(15, 0, 0);
    static {
      AutoAimRotationController.enableContinuousInput(-Math.PI, Math.PI);
    }
  }

  public static class PhotonVisionConstants {

    //TODO: tune stddev values
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

  public static class ShooterConstants {
    //TODO: find ball launch velocity
    /** units: m/s */
    public static final double launchVelocity = Units.feetToMeters(29); //convert from ft/s to m/s
    public static final double ratio = 1;
    public static final double shootHeight = Units.inchesToMeters(40);
  }

  public static class FieldConstants {
    public static final Map<DriverStation.Alliance, Pose2d> hubLocations = Map.of(
      Alliance.Blue, new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), new Rotation2d()),
      Alliance.Red, new Pose2d(Units.inchesToMeters(651.22 - 182.11), Units.inchesToMeters(158.84), new Rotation2d())
    );

    public static enum FieldZone {
        FIELD(Inches.of(651.22), Inches.of(317.69)),
        ALLIANCE(Inches.of(156.61), Inches.of(317.69)),
        TRENCH(Inches.of(120.0), Inches.of(49.96));

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

    /** Units:m/s^2 */
    public static final double g = 9.80;

    public static final double hubZ = Units.inchesToMeters(56.4);
  }
}
