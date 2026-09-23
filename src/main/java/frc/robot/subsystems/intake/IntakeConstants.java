package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;

public class IntakeConstants {

    public static final int PIVOT_MOTOR_ID = Robot.isReal() ? 6 : 36; // youre welcome Faye
    public static final int ROLLER_MOTOR_ID = Robot.isReal() ? 7 : 37;
    public static final int PIVOT_ENCODER_ID = 1;

    // motor configs
    public static final TalonFXConfiguration PIVOT_MOTOR_CONFIGURATION = new TalonFXConfiguration() {
        {
            Slot0.withKP(60.0).withKI(1.0).withKD(0.0).withKG(0.07).withGravityType(GravityTypeValue.Arm_Cosine);
            Feedback.withFeedbackRemoteSensorID(PIVOT_ENCODER_ID)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withSensorToMechanismRatio(2.0)
                .withRotorToSensorRatio(25.0);
            CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(30.0))
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(60.0));
        }
    };

    public static final CANcoderConfiguration PIVOT_ENCODER_CONFIGURATION = new CANcoderConfiguration() {
        {
            MagnetSensor.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(Degrees.of(0.0))
                .withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.9));
        }
    };

    // motion magic
    public static final MotionMagicConfigs PIVOT_FAST_MOTION_MAGIC_CONFIGURATION = new MotionMagicConfigs() {
        {
            withMotionMagicCruiseVelocity(RotationsPerSecond.of(200.0));
            withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(500.0));
            withMotionMagicJerk(RotationsPerSecondPerSecond.of(1600.0).per(Seconds));
        }
    };

    public static final MotionMagicConfigs PIVOT_SLOW_MOTION_MAGIC_CONFIGURATION = new MotionMagicConfigs() {
        {
            withMotionMagicCruiseVelocity(RotationsPerSecond.of(1.0));
            withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(5.0));
            withMotionMagicJerk(RotationsPerSecondPerSecond.of(1600.0).per(Seconds));
        }
    };

    public static final double ROLLER_GEAR_RATIO = 1.0 / 5.0;

    public static final TalonFXConfiguration ROLLER_MOTOR_CONFIGURATION = new TalonFXConfiguration() {
        {
            CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(20.0));
        }
    };

    public static enum IntakeState {
        DOWN_ON(Degrees.of(-9.0), 1.0),
        DOWN_OFF(Degrees.of(-9.0), 0.0),
        DOWN_REVERSE(Degrees.of(-9.0), -1.0),
        UP_OFF(Degrees.of(120.0), 0.0),
        BOUNCE_UP(Degrees.of(60.0), 0.0);

        public final Angle pivotAngle;
        public final double rollerDutyCycle;

        private IntakeState(Angle pivotAngle, double rollerDutyCycle) {
            this.pivotAngle = pivotAngle;
            this.rollerDutyCycle = rollerDutyCycle;
        }
    }
}
