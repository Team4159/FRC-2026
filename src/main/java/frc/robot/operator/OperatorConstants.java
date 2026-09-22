package frc.robot.operator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class OperatorConstants {

    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_CONTROLLER_PORT = 1;

    // controller joystick constants
    public static final double PRIMARY_TRANSLATION_DEADBAND = 0.05;
    public static final double PRIMARY_ROTATION_DEADBAND = 0.05;
    public static final double PRIMARY_TRANSLATION_EXPONENT = 2.0;
    public static final double PRIMARY_ROTATION_EXPONENT = 2.0;
    public static final double PRIMARY_TRANSLATION_RADIUS = 0.99;
    public static final double PRIMARY_ROTATION_RADIUS = 0.99;
    public static final double PRIMARY_TRIGGER_THRESHOLD = 0.1;

    public static enum DriveMode {
        TELEOP,
        BRAKE,
        POINT,
        IDLE,
    }

    public static enum DriveFlag {
        SLOW_MODE,
        DRIVE_ASSIST,
        AUTO_BRAKE,
        INTAKE_ASSIST,
        MANUAL_ALIGN,
    }

    // drive assist constants
    public static final Distance TRENCH_ASSIST_PASS_POSITION_TOLERANCE = Meters.of(0.45);
    public static final double TRENCH_ASSIST_APPROACH_INPUT_TO_TOLERANCE = 0.2;
    public static final Distance TRENCH_ASSIST_ALIGN_POSITION_INNER_TOLERANCE = Meters.of(0.05);
    public static final Distance TRENCH_ASSIST_ALIGN_POSITION_OUTER_TOLERANCE = Meters.of(0.15);
    public static final double TRENCH_ASSIST_ALIGN_STRENGTH = 0.8;
    public static final double TRENCH_ASSIST_ALIGN_INFLUENCE = 0.2;
    public static final Distance TRENCH_ASSIST_FRONT_PROTRUSION_EXTENT = Inches.of(10.0);

    // drive mode constants
    public static final Angle AUTO_BRAKE_REACHED_DESIRED_ANGLE_TOLERANCE = Degrees.of(5);

    public static final double SLOW_MODE_TRANSLATION_FACTOR = 0.25;
    public static final double SLOW_MODE_ROTATION_FACTOR = 1;

    public static final double INTAKE_ROTATION_INPUT_DEADZONE = 0.2;

    public static final double RADIAL_MODE_DEADBAND = 0.2;

    public static final double ALIGN_MODE_DEADBAND = 0.65;
    public static final double ALIGN_MODE_SPEED_TRANSLATION_FACTOR = 0.2;
    public static final double ALIGN_MODE_SPEED_ROTATION_FACTOR = 0.1;
}
