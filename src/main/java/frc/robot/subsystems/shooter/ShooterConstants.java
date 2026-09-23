package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;

public class ShooterConstants {

    public static final int LEFT_BOTTOM_MOTOR_ID = 9;
    public static final int LEFT_TOP_MOTOR_ID = 10;
    public static final int RIGHT_TOP_MOTOR_ID = 12;
    public static final int RIGHT_BOTTOM_MOTOR_ID = 11;

    // shooter motors config
    public static final TalonFXConfiguration MOTORS_CONFIGURATION = new TalonFXConfiguration() {
        {
            Slot0.withKP(7.5).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.25).withKA(2.14);
            CurrentLimits.withStatorCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(25.0));
            MotorOutput.withNeutralMode(NeutralModeValue.Coast);
            ClosedLoopRamps.withVoltageClosedLoopRampPeriod(Seconds.of(0.2));
        }
    };
    public static final TalonFXConfiguration RIGHT_MOTORS_CONFIGURATION = MOTORS_CONFIGURATION.clone();
    public static final TalonFXConfiguration LEFT_MOTORS_CONFIGURATION = MOTORS_CONFIGURATION.clone();

    static {
        RIGHT_MOTORS_CONFIGURATION.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        LEFT_MOTORS_CONFIGURATION.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    }

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

    public static final double FLYWHEEL_EFFICIENCY = 0.80;

    public static final Distance WHEEL_RADIUS = Inches.of(2.0);
    public static final Distance ROLLER_RADIUS = Inches.of(0.75);

    public static final double ROTOR_TO_WHEEL_RATIO = 1.0;
    public static final double ROTOR_TO_ROLLER_RATIO = 7.0 / 6.0;

    public static AngularVelocity VELOCITY_TOLERANCE = RPM.of(100.0);
    public static Angle HOOD_MAX_PITCH = Degrees.of(85.0);

    // robot relative shooter offset
    // TODO implement in the calculation
    public static final Transform2d FLYWHEEL_OFFSET = new Transform2d(0, 0, new Rotation2d());

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
