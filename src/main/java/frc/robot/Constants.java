// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import java.util.Map;
import java.util.Set;

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

    public static class FieldConstants {

        public static final Map<DriverStation.Alliance, Pose2d> HUB_LOCATIONS = Map.of(
            Alliance.Blue,
            new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), new Rotation2d()),
            Alliance.Red,
            new Pose2d(Units.inchesToMeters(651.22 - 182.11), Units.inchesToMeters(158.84), new Rotation2d())
        );

        public static final Set<Pose2d> HUB_LOB_LOCATIONS = Set.of(
            new Pose2d(2.5, 2.1, new Rotation2d()),
            new Pose2d(2.5, 5.6, new Rotation2d())
        );

        public static final Set<Pose2d> RED_LOB_LOCATIONS = Set.of(
            new Pose2d(14.1, 2.1, new Rotation2d()),
            new Pose2d(14.1, 5.6, new Rotation2d())
        );

        public static final Map<DriverStation.Alliance, Set<Pose2d>> LOB_LOCATIONS = Map.of(
            Alliance.Blue,
            HUB_LOB_LOCATIONS,
            Alliance.Red,
            RED_LOB_LOCATIONS
        );

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

        public static Distance FIELD_WIDTH = Inches.of(651.22);
        public static Distance FIELD_HEIGHT = Inches.of(317.69);

        public static Distance ALLIANCE_WIDTH = Inches.of(156.61);
        public static Distance ALLIANCE_HEIGHT = FIELD_HEIGHT;

        public static Distance TRENCH_X = Inches.of(182.11);
        public static Distance TRENCH_ZONE_WIDTH = Inches.of(140.0);
        public static Distance TRENCH_ZONE_HEIGHT = Inches.of(49.96);

        public static Distance TOWER_X = Inches.of(41.755);
        public static Distance TOWER_Y = Inches.of(147.47);
        public static Distance TOWER_WIDTH = Inches.of(35.2);

        public static Distance TRENCH_ZONE_Y_BUFFER = DrivetrainConstants.BUMPER_SIZE_Y.div(4);

        public static enum FieldZone {
            FIELD(
                new Translation2d(FIELD_WIDTH.div(2), FIELD_HEIGHT.div(2)),
                new Translation2d(FIELD_WIDTH, FIELD_HEIGHT)
            ),
            TRENCH_BLUE_LEFT(
                new Translation2d(
                    TRENCH_X,
                    FIELD_HEIGHT.minus(TRENCH_ZONE_HEIGHT.div(2)).minus(TRENCH_ZONE_Y_BUFFER.div(2))
                ),
                new Translation2d(TRENCH_X, FIELD_HEIGHT.minus(TRENCH_ZONE_HEIGHT.div(2))),
                new Translation2d(TRENCH_ZONE_WIDTH, TRENCH_ZONE_HEIGHT.plus(TRENCH_ZONE_Y_BUFFER))
            ),
            TRENCH_BLUE_RIGHT(
                new Translation2d(TRENCH_X, TRENCH_ZONE_HEIGHT.div(2).plus(TRENCH_ZONE_Y_BUFFER.div(2))),
                new Translation2d(TRENCH_X, TRENCH_ZONE_HEIGHT.div(2)),
                new Translation2d(TRENCH_ZONE_WIDTH, TRENCH_ZONE_HEIGHT.plus(TRENCH_ZONE_Y_BUFFER))
            ),
            TRENCH_RED_LEFT(
                new Translation2d(
                    FIELD_WIDTH.minus(TRENCH_X),
                    TRENCH_ZONE_HEIGHT.div(2).plus(TRENCH_ZONE_Y_BUFFER.div(2))
                ),
                new Translation2d(FIELD_WIDTH.minus(TRENCH_X), TRENCH_ZONE_HEIGHT.div(2)),
                new Translation2d(TRENCH_ZONE_WIDTH, TRENCH_ZONE_HEIGHT.plus(TRENCH_ZONE_Y_BUFFER))
            ),
            TRENCH_RED_RIGHT(
                new Translation2d(
                    FIELD_WIDTH.minus(TRENCH_X),
                    FIELD_HEIGHT.minus(TRENCH_ZONE_HEIGHT.div(2)).minus(TRENCH_ZONE_Y_BUFFER.div(2))
                ),
                new Translation2d(FIELD_WIDTH.minus(TRENCH_X), FIELD_HEIGHT.minus(TRENCH_ZONE_HEIGHT.div(2))),
                new Translation2d(TRENCH_ZONE_WIDTH, TRENCH_ZONE_HEIGHT.plus(TRENCH_ZONE_Y_BUFFER))
            );

            public final Translation2d CENTER, FOCUS, SIZE;

            private FieldZone(Translation2d center, Translation2d focus, Translation2d size) {
                this.CENTER = center;
                this.FOCUS = focus;
                this.SIZE = size;
            }

            private FieldZone(Translation2d center, Translation2d size) {
                this(center, center, size);
            }
        }

        public static final FieldZone[] TRENCH_ZONES = new FieldZone[] {
            FieldZone.TRENCH_BLUE_LEFT,
            FieldZone.TRENCH_BLUE_RIGHT,
            FieldZone.TRENCH_RED_LEFT,
            FieldZone.TRENCH_RED_RIGHT,
        };

        public static final double HUB_Z = Units.inchesToMeters(56.4);
    }

    public static class PhysicsConstants {

        /** Units:m/s^2 */
        public static final double GRAVITY = 9.80;
    }
}
