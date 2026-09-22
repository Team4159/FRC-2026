package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.Robot;
import java.util.Optional;

public class ShooterConstants {

    /** hood CAN ID */
    public static final int HOOD_MOTOR_ID = Robot.isReal() ? 8 : 38;

    //Hood PID Values
    public static final double HOOD_kP = 150;
    public static final double HOOD_kI = 25;
    public static final double HOOD_kD = 0;
    public static final double HOOD_kG = 0.03;
    // public static final double HOOD_kS = 5;

    // hood absolute encoder (WCP throughbore)
    /** Hood encoder CAN ID */
    public static final int HOOD_ENCODER_ID = 2;
    public static final Angle HOOD_ENCODER_OFFSET = Degrees.of(-242.65);
    public static final double HOOD_SENSOR_TO_MECHANISM_RATIO = 34 / 16; // evaluates to 2 instead of 2.125 but the lookup table is based off 2 so not changing
    /** ratio from the motor to the sensor (WCP throughbore encoder) */
    public static final double HOOD_ROTOR_TO_SENSOR_RATIO = 125;

    //Motion Magic®
    //think of it as max velocity
    public static final double HOOD_CRUISE_VELOCITY = 40;
    //the maximum acceleration used to achieve cruising velocity
    public static final double HOOD_ACCELERATION = 80;
    //maximum jerk (helps smooth the movement out more)
    public static final double HOOD_JERK = 1600;

    /** the angle between the center of the shooter and the very edge */
    public static final Angle HOOD_ANGLE_OFFSET = Degrees.of(7.6743605);

    public static final Angle HOOD_RESTING_ANGLE = Degrees.of(-5.8019605);

    // hood cancoder
    public static final CANcoderConfiguration CAN_CODER_CONFIG = new CANcoderConfiguration() {
        {
            MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
            MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
            MagnetSensor.withMagnetOffset(ShooterConstants.HOOD_ENCODER_OFFSET);
        }
    };

    // motion magic
    public static final MotionMagicConfigs HOOD_MOTION_MAGIC_CONFIG = new MotionMagicConfigs() {
        {
            MotionMagicCruiseVelocity = HOOD_CRUISE_VELOCITY; // Target cruise velocity of 80 rps
            MotionMagicAcceleration = HOOD_ACCELERATION; // Target acceleration of 160 rps/s (0.5 / seconds)
            MotionMagicJerk = HOOD_JERK; // Target jerk of 1600 rps/s/s (0.1 seconds)
        }
    };
    // hood motor conifg
    public static final TalonFXConfiguration HOOD_CONFIG = new TalonFXConfiguration() {
        {
            Slot0.kP = ShooterConstants.HOOD_kP;
            Slot0.kI = ShooterConstants.HOOD_kI;
            Slot0.kD = ShooterConstants.HOOD_kD;
            // Slot0.kS = ShooterConstants.kHoodS;
            // Slot0.kV = ShooterConstants.kHoodV;
            // Slot0.kA = ShooterConstants.kHoodA;
            Slot0.kG = ShooterConstants.HOOD_kG;
            Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
            MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            MotorOutput.NeutralMode = NeutralModeValue.Brake;

            CurrentLimits.SupplyCurrentLimitEnable = true;
            CurrentLimits.SupplyCurrentLimit = 20;
            // abs encoder
            Feedback.FeedbackRemoteSensorID = ShooterConstants.HOOD_ENCODER_ID;
            Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            Feedback.SensorToMechanismRatio = ShooterConstants.HOOD_SENSOR_TO_MECHANISM_RATIO;
            Feedback.RotorToSensorRatio = ShooterConstants.HOOD_ROTOR_TO_SENSOR_RATIO;
            MotionMagic = HOOD_MOTION_MAGIC_CONFIG;
        }
    };

    // Shooter Motor Config and PID
    // public static final double kP = 35;
    // public static final double kI = 10;
    public static final double SHOOTER_kP = 7.5;
    public static final double SHOOTER_kI = 0;
    public static final double SHOOTER_kD = 0;
    public static final double SHOOTER_kS = 0;
    public static final double SHOOTER_kV = 0.25;
    public static final double SHOOTER_kA = 2.14;

    public static final double SHOOTER_CURRENT_LIMIT = 25;
    public static final double SHOOTER_RAMP_PERIOD = 0.2;

    public static final int SHOOTER_LEFT_BOTTOM_MOTOR_ID = 9;
    public static final int SHOOTER_LEFT_TOP_MOTOR_ID = 10;
    public static final int SHOOTER_RIGHT_TOP_MOTOR_ID = 12;
    public static final int SHOOTER_RIGHT_BOTTOM_MOTOR_ID = 11;

    // shooter motors config
    public static final TalonFXConfiguration RIGHT_SHOOTER_MOTORS_CONFIG = new TalonFXConfiguration() {
        {
            Slot0.kP = ShooterConstants.SHOOTER_kP;
            Slot0.kI = ShooterConstants.SHOOTER_kI;
            Slot0.kD = ShooterConstants.SHOOTER_kD;
            Slot0.kS = ShooterConstants.SHOOTER_kS;
            Slot0.kV = ShooterConstants.SHOOTER_kV;
            Slot0.kA = ShooterConstants.SHOOTER_kA;
            CurrentLimits.SupplyCurrentLimitEnable = true;
            CurrentLimits.SupplyCurrentLimit = SHOOTER_CURRENT_LIMIT;
            MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            MotorOutput.NeutralMode = NeutralModeValue.Coast;
            ClosedLoopRamps.VoltageClosedLoopRampPeriod = SHOOTER_RAMP_PERIOD;
        }
    };

    public static final TalonFXConfiguration LEFT_SHOOTER_MOTORS_CONFIG = new TalonFXConfiguration() {
        {
            Slot0.kP = ShooterConstants.SHOOTER_kP;
            Slot0.kI = ShooterConstants.SHOOTER_kI;
            Slot0.kD = ShooterConstants.SHOOTER_kD;
            Slot0.kS = ShooterConstants.SHOOTER_kS;
            Slot0.kV = ShooterConstants.SHOOTER_kV;
            Slot0.kA = ShooterConstants.SHOOTER_kA;
            CurrentLimits.SupplyCurrentLimitEnable = true;
            CurrentLimits.SupplyCurrentLimit = SHOOTER_CURRENT_LIMIT;
            MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            MotorOutput.NeutralMode = NeutralModeValue.Coast;
            ClosedLoopRamps.VoltageClosedLoopRampPeriod = SHOOTER_RAMP_PERIOD;
        }
    };

    public static enum ShooterSetpoint {
        RESTING(RPM.of(0.0)),
        REV(RPM.of(1000.0)),
        LOB(RPM.of(2000.0)),
        FROM_HUB(RPM.of(2500.0), Degrees.of(75.0)),
        FROM_TOWER(RPM.of(3000.0), Degrees.of(70.0));

        public final AngularVelocity angularVelocity;
        public final Optional<Angle> pitch;

        private ShooterSetpoint(AngularVelocity angularVelocity, Angle pitch) {
            this.angularVelocity = angularVelocity;
            this.pitch = Optional.of(pitch);
        }

        private ShooterSetpoint(AngularVelocity angularVelocity) {
            this.angularVelocity = angularVelocity;
            this.pitch = Optional.empty();
        }
    }

    public static final Time BACKWARDS_TIME = Seconds.of(0.05);

    public static final double SHOOTER_EFFICIENCY = 0.80;

    public static final Distance SHOOTER_WHEEL_RADIUS = Inches.of(2.0);
    public static final Distance SHOOTER_ROLLER_RADIUS = Inches.of(0.75);

    public static final double ROTOR_TO_WHEEL_RATIO = 1.0;
    public static final double ROTOR_TO_ROLLER_RATIO = 7.0 / 6.0;

    public static AngularVelocity SHOOTER_VELOCITY_TOLERANCE = RPM.of(100.0);
    public static Angle HOOD_MAX_PITCH = Degrees.of(85.0);

    // robot relative shooter offset
    // TODO implement in the calculation
    public static final Transform2d SHOOTER_OFFSET = new Transform2d(0, 0, new Rotation2d());

    public static enum AutoShootStatus {
        SHOOT,
        OUT_OF_RANGE,
        WAITING,
    }

    // Old equation stuff
    // /** units: m/s */
    // public static final double SHOOTER_LAUNCH_VELOCITY = Units.feetToMeters(29); // convert from ft/s to m/s
    // public static final double SHOOTER_RATIO = 1;
    // public static final double SHOOTER_HEIGHT = Units.inchesToMeters(40);
}
