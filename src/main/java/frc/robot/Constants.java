// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
    public static final double kDriverControllerRotationDeadband = 0.1;
  }

  public static class DrivetrainConstants {
    public static final PhoenixPIDController AutoAimRotationController = new PhoenixPIDController(15, 0, 0);
    static {
      AutoAimRotationController.enableContinuousInput(-Math.PI, Math.PI);
    }
  }

  public static class PhotonVisionConstants{

    public static Transform3d leftShooterCamTransform = new Transform3d();
    public static Transform3d rightShooterCamTransform = new Transform3d();

  }

  public static class ShooterConstants {
    //TODO: find ball launch velocity
    /** units: m/s */
    public static final double launchVelocity = Units.feetToMeters(29);//convert from ft/s to m/s
    public static final double ratio = 1;
    public static final double shootHeight = Units.inchesToMeters(40);
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
}
