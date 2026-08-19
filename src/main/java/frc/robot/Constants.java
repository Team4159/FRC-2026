// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.generated.TunerConstants;
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

    public static class ClimberConstants {

        public static final double I_GAIN = 0;
        public static final double D_GAIN = 0;
        public static final double P_GAIN = 0;
        public static final int CLIMBER_ONE_MOTOR_ID = 15;
        //previously idClimberOne

        // public static final int idClimberTwo = 10;

        public static enum ClimberState {
            CLIMB(0.25),
            STOP(0),
            DOWN(-0.25);

            public double percentage;

            private ClimberState(double speed) {
                percentage = speed;
            }
        }
    }

    public static class HopperConstants {

        public static final int HOPPER_MOTOR_ID = 30;
        public static final Distance kHopperExtent = Inches.of(12.0);

        public static enum HopperState {
            FEED(1),
            REVERSE(-1),
            STOP(0);

            public double percentage;

            private HopperState(double speed) {
                percentage = speed;
            }
        }
    }

    public static class FeederConstants {

        public static final int FEEDER_MOTOR_ID = 20; // idk if this port is used yet plz check

        public static enum FeederState {
            FEED(1),
            UNSTUCKFEEDER(-1),
            STOP(0);

            public double percentage;

            private FeederState(double speed) {
                percentage = speed;
            }
        }
    }

    public static class IntakeConstants {

        public static final double ANGLE_P_GAIN = 60;
        public static final double ANGLE_I_GAIN = 1;
        public static final double ANGLE_D_GAIN = 0;
        public static final double ANGLE_G_GAIN = 0.07;

        // motion magic
        public static final double FAST_CRUISE_VELOCITY = 200;
        public static final double FAST_ACCELERATION = 500;
        public static final double FAST_JERK = 1600;

        public static final double SLOW_CRUISE_VELOCITY = 1;
        public static final double SLOW_ACCELERATION = 5;
        public static final double SLOW_JERK = 1600;

        public static final int ANGLE_ENCODER_ID = 1;
        public static final int INTAKE_ANGLE_MOTOR_ID = Robot.isReal() ? 6 : 36; // youre welcome Faye
        public static final int INTAKE_SPIN_MOTOR_ID = Robot.isReal() ? 7 : 37;

        public static final Angle ENCODER_OFFSET = Degrees.of(0);
        //now 25 and 2 because encoder is on the jackshaft now.
        public static final double MOTOR_TO_SENSOR_RATIO = 25;
        public static final double SENSEOR_TO_MECHANISM_RATIO = 2;

        // motor configs
        public static final CANcoderConfiguration CAN_CODER_CONFIG = new CANcoderConfiguration() {
            {
                MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.9));
                MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
                MagnetSensor.withMagnetOffset(IntakeConstants.ENCODER_OFFSET);
            }
        };

        public static final TalonFXConfiguration ANGLE_CONFIG = new TalonFXConfiguration() {
            {
                Slot0.kP = IntakeConstants.ANGLE_P_GAIN;
                Slot0.kI = IntakeConstants.ANGLE_I_GAIN;
                Slot0.kD = IntakeConstants.ANGLE_D_GAIN;
                Slot0.kG = IntakeConstants.ANGLE_G_GAIN;
                Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
                // abs encoder
                Feedback.FeedbackRemoteSensorID = IntakeConstants.ANGLE_ENCODER_ID;
                Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                Feedback.SensorToMechanismRatio = IntakeConstants.SENSEOR_TO_MECHANISM_RATIO;
                Feedback.RotorToSensorRatio = IntakeConstants.MOTOR_TO_SENSOR_RATIO;

                CurrentLimits.SupplyCurrentLimitEnable = true;
                CurrentLimits.SupplyCurrentLimit = 30;
                CurrentLimits.StatorCurrentLimitEnable = true;
                CurrentLimits.StatorCurrentLimit = 60;
            }
        };

        // motion magic
        public static final MotionMagicConfigs FAST_MOTION_MAGIC_CONFIG = new MotionMagicConfigs() {
            {
                MotionMagicCruiseVelocity = IntakeConstants.FAST_CRUISE_VELOCITY; // Target cruise velocity of 80 rps
                MotionMagicAcceleration = IntakeConstants.FAST_ACCELERATION; // Target acceleration of 160 rps/s (0.5
                // seconds)
                MotionMagicJerk = IntakeConstants.FAST_JERK; // Target jerk of 1600 rps/s/s (0.1 seconds)
            }
        };

        public static final MotionMagicConfigs SLOW_MOTION_MAGIC_CONFIG = new MotionMagicConfigs() {
            {
                MotionMagicCruiseVelocity = IntakeConstants.SLOW_CRUISE_VELOCITY;
                MotionMagicAcceleration = IntakeConstants.SLOW_ACCELERATION;
                MotionMagicJerk = IntakeConstants.SLOW_JERK;
            }
        };

        // public static final double kLocationGearRatio = 1.0 / 2.0;
        public static final double SPIN_GEAR_RATIO = 1.0 / 5.0;

        /** Units: rad/s */
        public static final double COMPRESS_RATE = 1;
        public static final double COMPRESS_P_GAIN = 1;
        public static final double COMPRESS_I_GAIN = 0;
        public static final double COMPRESS_D_GAIN = 0;
        public static final ProfiledPIDController COMPRESS_PID = new ProfiledPIDController(
            COMPRESS_P_GAIN,
            COMPRESS_I_GAIN,
            COMPRESS_D_GAIN,
            new TrapezoidProfile.Constraints(COMPRESS_RATE, 1)
        );

        public static enum IntakeState {
            DOWN_ON(Degrees.of(-9), 1),
            DOWN_OFF(Degrees.of(-9), 0),
            DOWN_REV(Degrees.of(-9), -1),
            UP_OFF(Degrees.of(120), 0),
            BOUNCE_UP(Degrees.of(60), 0),
            STOP(Degrees.of(120), 0);

            public final Angle rotationLocation;
            public final double spinSpeed;

            private IntakeState(Angle location, double speed) {
                rotationLocation = location;
                spinSpeed = speed;
            }
        }
    }

    public static class OperatorConstants {

        public static final int PRIMARY_CONTROLLER_PORT = 0;
        public static final int SECONDARY_CONTROLLER_PORT = 1;

        // controller joystick constants
        public static final double PRIMARY_TRANSLATION_DEADBAND = 0.05;
        public static final double PRIMARY_ROTATION_DEADBAND = 0.05;
        public static final double PRIMARY_TRANSLATION_EXPONENT = 2.0;
        public static final double PRIMARY_ROTATION_EXPONENT = 2.0;
        public static final double PRIMARY_TRANSLATION_RADIUS = 0.99;
        public static final double PRIMARY_ROTATION_RADIUS = 0.99;

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
        public static final double TRENCH_ASSIST_ALGIN_INFLUENCE = 0.2;
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

    public static class DrivetrainConstants {

        public static final Distance CHASSIS_SIZE_X = Inches.of(27.0);
        public static final Distance CHASSIS_SIZE_Y = Inches.of(27.0);

        public static final Distance BUMPER_SIZE_X = Inches.of(35.0);
        public static final Distance BUMPER_SIZE_Y = Inches.of(35.0);

        public static final double MAX_TRANSLATION_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MAX_ROTATION_SPEED = 2 * RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        public static final double POINT_P_GAIN = 5;
        public static final double POINT_I_GAIN = 0.0;
        public static final double POINT_D_GAIN = 0.0;
        public static final double POINT_FEED_FORWARD = 0.0;

        public static final double AIM_P_GAIN = 9;
        public static final double AIM_I_GAIN = 0.1;
        public static final double AIM_D_GAIN = 0.0;
        public static final double AIM_FEED_FORWARD = 0.0;

        public static final Angle AUTO_SHOOT_TOLERANCE = Degrees.of(10);
        public static final double AUTO_SHOOT_INPUT_MULTIPLIER = 1;

        public static final PhoenixPIDController AUTO_SHOOT_ROTATION_CONTROLLER = new PhoenixPIDController(
            AIM_P_GAIN,
            AIM_I_GAIN,
            AIM_D_GAIN
        );

        static {
            AUTO_SHOOT_ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        }
    }

    public static class ShooterConstants {

        //Hood PID Values

        //Proportional (this might be too high and contributing to the osclillations)
        public static final double HOOD_P_GAIN = 150;
        //Integral
        public static final double HOOD_I_GAIN = 25;
        //Derivative
        public static final double HOOD_D_GAIN = 0;
        //G - multiplied by cosine of the angle, so zero point should be when COM is perfectly horizontal
        public static final double HOOD_G_GAIN = 0.03;
        //S - static friction (this might be a bit high could also be contributing to some of the oscillations)
        public static final double HOOD_S_GAIN = 5;

        /** hood CAN ID */
        public static final int HOOD_MOTOR_ID = Robot.isReal() ? 8 : 38;

        // hood absolute encoder (WCP throughbore)
        /** Hood encoder CAN ID */
        public static final int HOOD_ENCODER_ID = 2;
        public static final Angle ENCODER_OFFSET = Degrees.of(-227.8 - 6.8 - 0.7 - 6 - 0.65 - 0.3 - 0.4);
        public static final double SENSOR_TO_MECHANISM_RATIO = 34 / 16;
        /** ratio from the motor to the sensor (WCP throughbore encoder) */
        public static final double MOTOR_TO_SENSOR_RATIO = 125;

        //Motion Magic®
        //think of it as max velocity
        public static final double CRUISE_VELOCITY = 40;
        //the maximum acceleration used to achieve cruising velocity
        public static final double ACCELERATION = 80;
        //maximum jerk (helps smooth the movement out more)
        public static final double JERK = 1600;

        /** the angle between the center of the shooter and the very edge */
        public static final Angle HOOD_ANGLE_OFFSET = Degrees.of(7.6743605);

        public static final Angle RESTING_ANGLE = Degrees.of(-5.8019605);

        // hood cancoder
        public static final CANcoderConfiguration CAN_CODER_CONFIG = new CANcoderConfiguration() {
            {
                MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
                MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
                MagnetSensor.withMagnetOffset(ShooterConstants.ENCODER_OFFSET);
            }
        };

        // motion magic
        public static final MotionMagicConfigs HOOD_MOTION_MAGIC_CONFIG = new MotionMagicConfigs() {
            {
                MotionMagicCruiseVelocity = ShooterConstants.CRUISE_VELOCITY; // Target cruise velocity of 80 rps
                MotionMagicAcceleration = ShooterConstants.ACCELERATION; // Target acceleration of 160 rps/s (0.5
                // seconds)
                MotionMagicJerk = ShooterConstants.JERK; // Target jerk of 1600 rps/s/s (0.1 seconds)
            }
        };
        // hood motor conifg
        public static final TalonFXConfiguration HOOD_CONFIG = new TalonFXConfiguration() {
            {
                Slot0.kP = ShooterConstants.HOOD_P_GAIN;
                Slot0.kI = ShooterConstants.HOOD_I_GAIN;
                Slot0.kD = ShooterConstants.HOOD_D_GAIN;
                // Slot0.kS = ShooterConstants.kHoodS;
                // Slot0.kV = ShooterConstants.kHoodV;
                // Slot0.kA = ShooterConstants.kHoodA;
                Slot0.kG = ShooterConstants.HOOD_G_GAIN;
                Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
                MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                MotorOutput.NeutralMode = NeutralModeValue.Brake;

                CurrentLimits.SupplyCurrentLimitEnable = true;
                CurrentLimits.SupplyCurrentLimit = 20;
                // abs encoder
                Feedback.FeedbackRemoteSensorID = ShooterConstants.HOOD_ENCODER_ID;
                Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                Feedback.SensorToMechanismRatio = ShooterConstants.SENSOR_TO_MECHANISM_RATIO;
                Feedback.RotorToSensorRatio = ShooterConstants.MOTOR_TO_SENSOR_RATIO;
                MotionMagic = HOOD_MOTION_MAGIC_CONFIG;
            }
        };

        // Shooter Motor Config and PID
        // public static final double kP = 35;
        // public static final double kI = 10;
        public static final double P_GAIN = 7.5;
        public static final double I_GAIN = 0;
        public static final double D_GAIN = 0;
        public static final double S_GAIN = 0;
        //remove if doesnt work
        public static final double V_GAIN = 0.25;
        public static final double A_GAIN = 2.14;

        public static final double CURRENT_LIMIT = 25;
        public static final double RAMP_RATE = 0.2;

        public static final int SHOOTER_LEFT_BOTTOM_MOTOR_ID = 9;
        public static final int SHOOTER_LEFT_TOP_MOTOR_ID = 10;
        public static final int SHOOTER_RIGHT_TOP_MOTOR_ID = 12;
        public static final int SHOOTER_RIGHT_BOTTOM_MOTOR_ID = 11;

        // shooter motors config
        public static final TalonFXConfiguration RIGHT_SHOOTER_MOTORS_CONFIG = new TalonFXConfiguration() {
            {
                Slot0.kP = ShooterConstants.P_GAIN;
                Slot0.kI = ShooterConstants.I_GAIN;
                Slot0.kD = ShooterConstants.D_GAIN;
                Slot0.kS = ShooterConstants.S_GAIN;
                Slot0.kV = ShooterConstants.V_GAIN;
                Slot0.kA = ShooterConstants.A_GAIN;
                CurrentLimits.SupplyCurrentLimitEnable = true;
                CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
                MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                MotorOutput.NeutralMode = NeutralModeValue.Coast;
                ClosedLoopRamps.VoltageClosedLoopRampPeriod = RAMP_RATE;
            }
        };

        public static final TalonFXConfiguration LEFT_SHOOTER_MOTORS_CONFIG = new TalonFXConfiguration() {
            {
                Slot0.kP = ShooterConstants.P_GAIN;
                Slot0.kI = ShooterConstants.I_GAIN;
                Slot0.kD = ShooterConstants.D_GAIN;
                Slot0.kS = ShooterConstants.S_GAIN;
                Slot0.kV = ShooterConstants.V_GAIN;
                Slot0.kA = ShooterConstants.A_GAIN;
                CurrentLimits.SupplyCurrentLimitEnable = true;
                CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
                MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                MotorOutput.NeutralMode = NeutralModeValue.Coast;
                ClosedLoopRamps.VoltageClosedLoopRampPeriod = RAMP_RATE;
            }
        };

        public static final AngularVelocity SHOOTER_ANGULAR_VELOCITY = RPM.of(2000);
        public static final AngularVelocity LOB_ANGULAR_VELOCITY = RPM.of(2000);
        public static final AngularVelocity RESTING_ANGULAR_VELOCITY = RPM.of(1000);
        public static final AngularVelocity HUB_ANGULAR_VELOCITY = RPM.of(2500);
        public static final AngularVelocity TOWER_ANGULAR_VELOCITY = RPM.of(3000);

        public static final Angle HUB_HOOD_PITCH = Degrees.of(75);
        public static final Angle TOWER_HOOD_PITCH = Degrees.of(70);

        public static final double BACKWARDS_TIME = 0.05;

        // Old equation stuff
        /** units: m/s */
        public static final double LAUNCH_VELOCITY = Units.feetToMeters(29); // convert from ft/s to m/s
        public static final double RATIO = 1;
        public static final double SHOOTER_HEIGHT = Units.inchesToMeters(40);

        public static AngularVelocity SHOOTER_VELOCITY_TOLERANCE = RPM.of(100);
        public static Angle MAX_PITCH = Degrees.of(85);

        public static final Distance SHOOTER_WHEEL_RADIUS = Inches.of(2);
        public static final Distance SHOOTER_ROLLER_RADIUS = Inches.of(0.75);

        public static final double MOTOR_TO_WHEEL_RATIO = 1;
        public static final double MOTOR_TO_ROLLER_RATIO = 1.167;

        public static final double SHOOTER_EFFICIENCY = 0.80;

        // robot relative shooter offset
        // TODO implement in the calculation
        public static final Transform2d SHOOTER_OFFSET = new Transform2d(0, 0, new Rotation2d());

        public static enum AutoShootStatus {
            SHOOT(LEDConstants.LEDStatus.GREEN_BLINK),
            OUTOFRANGE(LEDConstants.LEDStatus.RED_BLINK),
            WAITING(LEDConstants.LEDStatus.YELLOW_BLINK);

            public LEDConstants.LEDStatus ledStatus;

            private AutoShootStatus(LEDConstants.LEDStatus ledStatus) {
                this.ledStatus = ledStatus;
            }
        }
    }

    public static class PhotonVisionConstants {

        // TODO: tune stddev values
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1, 1, 4);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.25, 0.25, 1);

        public static final Transform3d LEFT_SHOOTER_CAM_TRANSFORM = new Transform3d(
            Units.inchesToMeters(-1.2887),
            Units.inchesToMeters(8.8466),
            Units.inchesToMeters(21.1190),
            new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(-5))
        );
        public static final Transform3d RIGHT_SHOOTER_CAM_TRANSFORM = new Transform3d(
            Units.inchesToMeters(-1.2887),
            Units.inchesToMeters(-8.8466),
            Units.inchesToMeters(21.1190),
            new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(5))
        );
    }

    public static class FieldConstants {

        public static final Map<DriverStation.Alliance, Pose2d> HUB_LOCATIONS = Map.of(
            Alliance.Blue,
            new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), new Rotation2d()),
            Alliance.Red,
            new Pose2d(Units.inchesToMeters(651.22 - 182.11), Units.inchesToMeters(158.84), new Rotation2d())
        );

        public static final Set<Pose2d> HUB_LOB_POSITIONS = Set.of(
            new Pose2d(2.5, 2.1, new Rotation2d()),
            new Pose2d(2.5, 5.6, new Rotation2d())
        );

        public static final Set<Pose2d> RED_LOB_POSITIONS = Set.of(
            new Pose2d(14.1, 2.1, new Rotation2d()),
            new Pose2d(14.1, 5.6, new Rotation2d())
        );

        public static final Map<DriverStation.Alliance, Set<Pose2d>> LOB_LOCATIONS = Map.of(
            Alliance.Blue,
            HUB_LOB_POSITIONS,
            Alliance.Red,
            RED_LOB_POSITIONS
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

        /** Units:m/s^2 */
        public static final double G = 9.80;

        public static final double HUB_Z = Units.inchesToMeters(56.4);
    }

    public static class AutoConstants {

        /** units: seconds */
        public static final double SHOOT_TIME = 4;

        public static final APConstraints AUTOPILOT_CONSTRAINTS = new APConstraints()
            .withAcceleration(7.0)
            .withJerk(3.5);
        public static final APProfile AUTOPILOT_ALIGN_PROFILE = new APProfile(AUTOPILOT_CONSTRAINTS)
            .withErrorXY(Centimeters.of(2.0))
            .withErrorTheta(Degrees.of(1))
            .withBeelineRadius(Centimeters.of(5.0));
        public static final Autopilot AUTOPILOT_ALIGN_CONTROLELR = new Autopilot(AUTOPILOT_ALIGN_PROFILE);
        public static final APProfile AUTOPILOT_CRUISE_PROFILE = new APProfile(AUTOPILOT_CONSTRAINTS)
            .withErrorXY(Centimeters.of(20.0))
            .withErrorTheta(Degrees.of(15.0))
            .withBeelineRadius(Centimeters.of(25.0));
        public static final Autopilot AUTOPILOT_CRUISE_CONTROLLER = new Autopilot(AUTOPILOT_CRUISE_PROFILE);
    }

    public static class AlignConstants {

        public static enum TowerAlignGoal {
            LEFT(
                new APTarget(
                    new Pose2d(
                        FieldConstants.TOWER_X,
                        FieldConstants.TOWER_Y
                            .plus(FieldConstants.TOWER_WIDTH.div(2))
                            .plus(DrivetrainConstants.BUMPER_SIZE_X.div(2)),
                        Rotation2d.kZero
                    )
                )
                    .withVelocity(0)
                    .withoutEntryAngle()
            ),
            RIGHT(
                new APTarget(
                    new Pose2d(
                        FieldConstants.TOWER_X,
                        FieldConstants.TOWER_Y
                            .minus(FieldConstants.TOWER_WIDTH.div(2))
                            .minus(DrivetrainConstants.BUMPER_SIZE_X.div(2)),
                        Rotation2d.k180deg
                    )
                )
                    .withVelocity(0)
                    .withEntryAngle(Rotation2d.k180deg)
            ),
            MIDDLE_FRONT(
                new APTarget(
                    new Pose2d(
                        FieldConstants.TOWER_X.plus(DrivetrainConstants.BUMPER_SIZE_X.div(2)).plus(Inches.of(6)),
                        FieldConstants.TOWER_Y,
                        Rotation2d.k180deg
                    )
                )
                    .withVelocity(0)
                    .withoutEntryAngle(),
                new APTarget(new Pose2d(FieldConstants.TOWER_X, FieldConstants.TOWER_Y, Rotation2d.k180deg))
                    .withVelocity(0)
                    .withoutEntryAngle()
            ),
            MIDDLE_BACK(
                new APTarget(
                    new Pose2d(
                        FieldConstants.TOWER_X.minus(DrivetrainConstants.BUMPER_SIZE_X.div(2)).minus(Inches.of(6)),
                        FieldConstants.TOWER_Y,
                        Rotation2d.kZero
                    )
                )
                    .withVelocity(0)
                    .withoutEntryAngle(),
                new APTarget(new Pose2d(FieldConstants.TOWER_X, FieldConstants.TOWER_Y, Rotation2d.kZero))
                    .withVelocity(0)
                    .withoutEntryAngle()
            );

            public final APTarget[] targets;

            private TowerAlignGoal(APTarget... targets) {
                this.targets = targets;
            }
        }
    }

    public static final class LEDConstants {

        private static final Distance LED_SPACING = Meters.of(1.0 / 120.0);

        public static enum LEDStatus {
            RAINBOW(LEDPattern.rainbow(255, 64)),
            RAINBOW_SCROLL(RAINBOW.pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LED_SPACING)),

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

    public static final class JoeLookupTableConstants {

        public static record LookupTablePoint(AngularVelocity angularVelocity, double efficiency) {}

        /**
         * adjust the angle of the hood down by this much (in radians for each
         * meter/second slow the calculated tangential speed is
         * multiplied by the distance from the hub (higher distance needs more
         * correction)
         */
        public static final double SHOOTER_DISTANCE_VELOCITY_CORRECTION = 0.01;

        public static final Distance MAX_DISTANCE = Meters.of(4.5);

        // stores desired motor angular velocity and shooter efficiency based on position
        public static final Map<Distance, LookupTablePoint> JOE_LOOKUP_TABLE = Map.ofEntries(
            //non-continuous feeding
            // Map.entry(Meters.of(1),   new LookupTablePoint(RPM.of(2000), 0.93)),
            // Map.entry(Meters.of(1.5), new LookupTablePoint(RPM.of(2100), 0.92)),
            // Map.entry(Meters.of(2),   new LookupTablePoint(RPM.of(2200), 0.91)),
            // Map.entry(Meters.of(2.5), new LookupTablePoint(RPM.of(2300), 0.90)),
            // Map.entry(Meters.of(3),   new LookupTablePoint(RPM.of(2400), 0.88)),
            // Map.entry(Meters.of(3.5), new LookupTablePoint(RPM.of(2500), 0.84)),
            // Map.entry(Meters.of(4),   new LookupTablePoint(RPM.of(2700), 0.78)),
            // Map.entry(Meters.of(4.5), new LookupTablePoint(RPM.of(2900), 0.72))
            //before gear ratio fix
            // Map.entry(Meters.of(1),   new LookupTablePoint(RPM.of(1800), 1.1)),
            // Map.entry(Meters.of(1.5), new LookupTablePoint(RPM.of(1900), 1.05)),
            // Map.entry(Meters.of(2),   new LookupTablePoint(RPM.of(2000), 1.025)),
            // Map.entry(Meters.of(2.5), new LookupTablePoint(RPM.of(2100), 1)),
            // Map.entry(Meters.of(3),   new LookupTablePoint(RPM.of(2200), 1)),
            // Map.entry(Meters.of(3.5), new LookupTablePoint(RPM.of(2300), 0.97)),
            // Map.entry(Meters.of(4),   new LookupTablePoint(RPM.of(2500), 0.90)),
            // Map.entry(Meters.of(4.5), new LookupTablePoint(RPM.of(2700), 0.85))
            // Map.entry(Meters.of(1),   new LookupTablePoint(RPM.of(1800), 0.92)),
            // Map.entry(Meters.of(1.5), new LookupTablePoint(RPM.of(1900), 0.90)),
            // Map.entry(Meters.of(2),   new LookupTablePoint(RPM.of(2000), 0.88)),
            // Map.entry(Meters.of(2.5), new LookupTablePoint(RPM.of(2100), 0.85)),
            // Map.entry(Meters.of(3),   new LookupTablePoint(RPM.of(2200), 0.80)),
            // Map.entry(Meters.of(3.5), new LookupTablePoint(RPM.of(2300), 0.78)),
            // Map.entry(Meters.of(4),   new LookupTablePoint(RPM.of(2500), 0.73)),
            // Map.entry(Meters.of(4.5), new LookupTablePoint(RPM.of(2700), 0.70))
            // Map.entry(Meters.of(1), new LookupTablePoint(RPM.of(1800), 1.20)), //1
            // Map.entry(Meters.of(1.5), new LookupTablePoint(RPM.of(1900), 1.20)),
            // Map.entry(Meters.of(2), new LookupTablePoint(RPM.of(2000), 1.15)), //2
            // Map.entry(Meters.of(2.5), new LookupTablePoint(RPM.of(2100), 1.05)),
            // Map.entry(Meters.of(3), new LookupTablePoint(RPM.of(2200), 1.00)), //3
            // Map.entry(Meters.of(3.5), new LookupTablePoint(RPM.of(2300), 0.95)),
            // Map.entry(Meters.of(4), new LookupTablePoint(RPM.of(2500), 0.90)),
            // Map.entry(Meters.of(4.5), new LookupTablePoint(RPM.of(2700), 0.85))
            Map.entry(Meters.of(1.0), new LookupTablePoint(RPM.of(1800), 1.1)),
            Map.entry(Meters.of(1.5), new LookupTablePoint(RPM.of(1900), 1.05)),
            Map.entry(Meters.of(2.0), new LookupTablePoint(RPM.of(2000), 1.025)),
            Map.entry(Meters.of(2.5), new LookupTablePoint(RPM.of(2100), 1)),
            Map.entry(Meters.of(3.0), new LookupTablePoint(RPM.of(2200), 1)),
            Map.entry(Meters.of(3.5), new LookupTablePoint(RPM.of(2300), 0.97)),
            Map.entry(Meters.of(4.0), new LookupTablePoint(RPM.of(2500), 0.90)),
            Map.entry(Meters.of(4.5), new LookupTablePoint(RPM.of(2700), 0.85))
        );
    }
}
