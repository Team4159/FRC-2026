package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class DrivetrainConstants {

    public static final double MAX_TRANSLATION_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ROTATION_SPEED = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    public static final Distance CHASSIS_SIZE_X = Inches.of(27.0);
    public static final Distance CHASSIS_SIZE_Y = Inches.of(27.0);

    public static final Distance BUMPER_SIZE_X = Inches.of(35.0);
    public static final Distance BUMPER_SIZE_Y = Inches.of(35.0);

    public static final double POINT_kP = 5;
    public static final double POINT_kI = 0.0;
    public static final double POINT_kD = 0.0;
    public static final double POINT_FEED_FORWARD = 0.0;

    public static final Angle AUTO_SHOOT_TOLERANCE = Degrees.of(10.0);
    public static final double AUTO_SHOOT_INPUT_MULTIPLIER = 1.0;
    public static final double AUTO_SHOOT_FEED_FORWARD = 0.0;
    public static final PhoenixPIDController AUTO_SHOOT_ROTATION_CONTROLLER = new PhoenixPIDController(9, 0.1, 0);

    static {
        AUTO_SHOOT_ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    }
}
