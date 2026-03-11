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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;

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
        public static final Angle kPrimaryAutoBrakeReachedDesiredAngleTolerance = Degrees.of(0.5);

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

        public static final PhoenixPIDController AutoAimRotationController = new PhoenixPIDController(kAimKP, kAimKI,
                kAimKD);
        static {
            AutoAimRotationController.enableContinuousInput(-Math.PI, Math.PI);
        }

    }

    public static class ClimberConstants {
        public static final double kAngleToPosition = 0.0013546667;

        public static final double kZeroDutyCycle = 0.05;
        public static final double kZeroLiftDuration = 0.5;

        public static enum ClimberState {
            LOWERED(Meters.of(0.0)),
            EXTENDED_FULL(Meters.of(0.0)),
            EXTENDED_PARTIAL(Meters.of(0.0));

            public final Distance position;

            private ClimberState(Distance position) {
                this.position = position;
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
                        Units.degreesToRadians(-5)));
    }

    public static class ShooterConstants {
        // TODO: find ball launch velocity
        /** units: m/s */
        public static final double launchVelocity = Units.feetToMeters(29); // convert from ft/s to m/s
        public static final double ratio = 1;
        public static final double shootHeight = Units.inchesToMeters(40);
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
}
