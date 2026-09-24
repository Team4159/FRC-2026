package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class FlywheelConstants {

    public static final TalonFX LEFT_BOTTOM_MOTOR = new TalonFX(9);
    public static final TalonFX LEFT_TOP_MOTOR = new TalonFX(10);
    public static final TalonFX RIGHT_BOTTOM_MOTOR = new TalonFX(11);
    public static final TalonFX RIGHT_TOP_MOTOR = new TalonFX(12);
    public static final TalonFX[] MOTORS = { LEFT_BOTTOM_MOTOR, LEFT_TOP_MOTOR, RIGHT_BOTTOM_MOTOR, RIGHT_TOP_MOTOR };

    // shooter motors config
    public static final TalonFXConfiguration MOTORS_CONFIGURATION = new TalonFXConfiguration() {
        {
            Slot0.withKP(7.5).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.25).withKA(2.14);
            CurrentLimits.withStatorCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(25.0));
            MotorOutput.withNeutralMode(NeutralModeValue.Coast);
            ClosedLoopRamps.withVoltageClosedLoopRampPeriod(Seconds.of(0.2));
        }
    };

    public static final double SHOOT_EFFICIENCY = 0.80;

    public static final Distance WHEEL_RADIUS = Inches.of(2.0);
    public static final Distance ROLLER_RADIUS = Inches.of(0.75);

    public static final double ROTOR_TO_WHEEL_RATIO = 1.0 / 1.0;
    public static final double ROTOR_TO_ROLLER_RATIO = 7.0 / 6.0;

    public static final AngularVelocity VELOCITY_TARGET_TOLERANCE = RPM.of(50.0);

    // robot relative shooter offset
    // TODO implement in the calculation
    public static final Transform2d EXIT_OFFSET = new Transform2d(0, 0, new Rotation2d());

    // Old equation stuff
    // /** units: m/s */
    // public static final double SHOOTER_LAUNCH_VELOCITY = Units.feetToMeters(29); // convert from ft/s to m/s
    // public static final double SHOOTER_RATIO = 1;
    // public static final double SHOOTER_HEIGHT = Units.inchesToMeters(40);

    static {
        TalonFXConfiguration LEFT_MOTORS_CONFIGURATION = MOTORS_CONFIGURATION.clone();
        LEFT_MOTORS_CONFIGURATION.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        LEFT_BOTTOM_MOTOR.getConfigurator().apply(LEFT_MOTORS_CONFIGURATION);
        LEFT_TOP_MOTOR.getConfigurator().apply(LEFT_MOTORS_CONFIGURATION);
        TalonFXConfiguration RIGHT_MOTORS_CONFIGURATION = MOTORS_CONFIGURATION.clone();
        RIGHT_MOTORS_CONFIGURATION.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        RIGHT_BOTTOM_MOTOR.getConfigurator().apply(RIGHT_MOTORS_CONFIGURATION);
        RIGHT_TOP_MOTOR.getConfigurator().apply(RIGHT_MOTORS_CONFIGURATION);
    }
}
