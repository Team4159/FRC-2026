package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;

public class IntakeConstants {

    public static final int ANGLE_ENCODER_ID = 1;
    public static final int ANGLE_MOTOR_ID = Robot.isReal() ? 6 : 36; // youre welcome Faye
    public static final int SPIN_MOTOR_ID = Robot.isReal() ? 7 : 37;

    public static final Angle ANGLE_ENCODER_OFFSET = Degrees.of(0);

    public static final double ANGLE_kP = 60;
    public static final double ANGLE_kI = 1;
    public static final double ANGLE_kD = 0;
    public static final double ANGLE_kG = 0.07;

    // motion magic
    public static final double ANGLE_FAST_CRUISE_VELOCITY = 200;
    public static final double ANGLE_FAST_ACCELERATION = 500;
    public static final double ANGLE_FAST_JERK = 1600;

    public static final double ANGLE_SLOW_CRUISE_VELOCITY = 1;
    public static final double ANGLE_SLOW_ACCELERATION = 5;
    public static final double ANGLE_SLOW_JERK = 1600;

    // now 25 and 2 because encoder is on the jackshaft now.
    public static final double ANGLE_ROTOR_TO_SENSOR_RATIO = 25;
    public static final double ANGLE_SENSOR_TO_MECHANISM_RATIO = 2;

    // motor configs
    public static final CANcoderConfiguration ANGLE_CAN_CODER_CONFIG = new CANcoderConfiguration() {
        {
            MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.9));
            MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
            MagnetSensor.withMagnetOffset(ANGLE_ENCODER_OFFSET);
        }
    };

    public static final TalonFXConfiguration ANGLE_CONFIG = new TalonFXConfiguration() {
        {
            Slot0.kP = ANGLE_kP;
            Slot0.kI = ANGLE_kI;
            Slot0.kD = ANGLE_kD;
            Slot0.kG = ANGLE_kG;
            Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
            // abs encoder
            Feedback.FeedbackRemoteSensorID = ANGLE_ENCODER_ID;
            Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            Feedback.SensorToMechanismRatio = ANGLE_SENSOR_TO_MECHANISM_RATIO;
            Feedback.RotorToSensorRatio = ANGLE_ROTOR_TO_SENSOR_RATIO;

            CurrentLimits.SupplyCurrentLimitEnable = true;
            CurrentLimits.SupplyCurrentLimit = 30;
            CurrentLimits.StatorCurrentLimitEnable = true;
            CurrentLimits.StatorCurrentLimit = 60;
        }
    };

    // motion magic
    public static final MotionMagicConfigs ANGLE_FAST_MOTION_MAGIC_CONFIG = new MotionMagicConfigs() {
        {
            MotionMagicCruiseVelocity = ANGLE_FAST_CRUISE_VELOCITY; // Target cruise velocity of 80 rps
            MotionMagicAcceleration = ANGLE_FAST_ACCELERATION; // Target acceleration of 160 rps/s (0.5 / seconds)
            MotionMagicJerk = ANGLE_FAST_JERK; // Target jerk of 1600 rps/s/s (0.1 seconds)
        }
    };

    public static final MotionMagicConfigs ANGLE_SLOW_MOTION_MAGIC_CONFIG = new MotionMagicConfigs() {
        {
            MotionMagicCruiseVelocity = ANGLE_SLOW_CRUISE_VELOCITY;
            MotionMagicAcceleration = ANGLE_SLOW_ACCELERATION;
            MotionMagicJerk = ANGLE_SLOW_JERK;
        }
    };

    // public static final double kLocationGearRatio = 1.0 / 2.0;
    public static final double SPIN_GEAR_RATIO = 1.0 / 5.0;

    public static enum IntakeState {
        DOWN_ON(Degrees.of(-9), 1),
        DOWN_OFF(Degrees.of(-9), 0),
        DOWN_REVERSE(Degrees.of(-9), -1),
        UP_OFF(Degrees.of(120), 0),
        BOUNCE_UP(Degrees.of(60), 0);

        public final Angle angleLocation;
        public final double spinDutyCycle;

        private IntakeState(Angle angleLocation, double spinDutyCycle) {
            this.angleLocation = angleLocation;
            this.spinDutyCycle = spinDutyCycle;
        }
    }
}
