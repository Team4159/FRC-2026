package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;

public class HoodConstants {

    public static final TalonFX MOTOR = new TalonFX(Robot.isReal() ? 8 : 38);
    public static final CANcoder ENCODER = new CANcoder(2);

    public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration() {
        {
            Slot0.withKP(150.0)
                .withKI(25.0)
                .withKD(0.0)
                .withKG(0.03)
                // .withKS(5.0)
                .withGravityType(GravityTypeValue.Arm_Cosine);
            MotorOutput.withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
            CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(20.0));
            Feedback.withFeedbackRemoteSensorID(ENCODER.getDeviceID())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withSensorToMechanismRatio((double) (34 / 16))
                .withRotorToSensorRatio(125.0);
            MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(40.0))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(80.0))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.of(1600.0).per(Seconds));
        }
    };

    public static final CANcoderConfiguration ENCODER_CONFIGURATION = new CANcoderConfiguration() {
        {
            MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
            MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
            MagnetSensor.withMagnetOffset(Degrees.of(-242.65));
        }
    };

    public static final Angle PITCH_OFFSET = Degrees.of(7.6743605);
    public static final Angle RESTING_PITCH = Degrees.of(-5.8019605);
    public static final Angle MAX_PITCH = Degrees.of(85.0);

    static {
        MOTOR.getConfigurator().apply(MOTOR_CONFIGURATION);
        ENCODER.getConfigurator().apply(ENCODER_CONFIGURATION);
    }
}
