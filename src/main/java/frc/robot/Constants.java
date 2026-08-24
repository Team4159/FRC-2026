// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
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

        public static final int PRIMARY_CONTROLLER_PORT = 0;
        public static final double PRIMARY_TRANSLATION_DEADBAND = 0.05;
        public static final double PRIMARY_ROTATION_DEADBAND = 0.05;
        public static final double PRIMARY_TRANSLATION_EXPONENT = 2.0;
        public static final double PRIMARY_ROTATION_EXPONENT = 2.0;
    }

    public static class DrivetrainConstants {

        public static final LinearVelocity MAX_TRANSLATION_SPEED = TunerConstants.kSpeedAt12Volts.times(0.3);
        public static final AngularVelocity MAX_ROTATION_SPEED = RotationsPerSecond.of(0.75);
    }
}
