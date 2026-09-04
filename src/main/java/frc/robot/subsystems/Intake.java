package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;

public class Intake extends SubsystemBase {

    private final TalonFX locationMotor;
    private final TalonFX spinMotor;
    private final CANcoder canCoder;

    private double rollerPercentage;

    private final MotionMagicVoltage intakeMotionMagicVoltage;

    // private final VelocityVoltage intakeVelocityVoltage;

    public Intake() {
        locationMotor = new TalonFX(IntakeConstants.ANGLE_MOTOR_ID);
        spinMotor = new TalonFX(IntakeConstants.SPIN_MOTOR_ID);
        canCoder = new CANcoder(IntakeConstants.ANGLE_ENCODER_ID);

        canCoder.getConfigurator().apply(IntakeConstants.ANGLE_CAN_CODER_CONFIG);
        locationMotor.getConfigurator().apply(IntakeConstants.ANGLE_CONFIG);
        setMotionMagic(IntakeConstants.ANGLE_FAST_MOTION_MAGIC_CONFIG);

        intakeMotionMagicVoltage = new MotionMagicVoltage(0);
        setLocation(IntakeState.DOWN_OFF.angleLocation);
        // intakeVelocityVoltage = new VelocityVoltage(0);

        CurrentLimitsConfigs rollerCurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Amps.of(20))
            .withSupplyCurrentLimitEnable(true);
        spinMotor.getConfigurator().apply(rollerCurrentLimits);

        rollerPercentage = 0;
    }

    public void setMotionMagic(MotionMagicConfigs motionMagicConfigs) {
        locationMotor.getConfigurator().apply(IntakeConstants.ANGLE_CONFIG.withMotionMagic(motionMagicConfigs));
    }

    public void setSpinSpeed(double speed) {
        spinMotor.set(speed);
    }

    public void setLocation(Angle angle) {
        locationMotor.setControl(intakeMotionMagicVoltage.withPosition(angle));
    }

    private Angle getPivotAngle() {
        return Rotations.of(locationMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake angle", getPivotAngle().in(Degrees));
        SmartDashboard.putNumber(
            "intake pid error",
            Units.rotationsToDegrees(locationMotor.getClosedLoopError().getValueAsDouble())
        );

        if (getPivotAngle().in(Degrees) < 15) {
            setSpinSpeed(rollerPercentage);
        } else {
            setSpinSpeed(0);
        }
    }

    public class ChangeStates extends Command {

        private IntakeState state;

        public ChangeStates(IntakeState state) {
            this.state = state;
            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            rollerPercentage = state.spinDutyCycle;
            setLocation(state.angleLocation);
        }

        @Override
        public void end(boolean interrupt) {
            Intake.this.setSpinSpeed(IntakeState.STOP.spinDutyCycle);
            rollerPercentage = 0;
        }
    }

    public class CompressIntake extends Command {

        public CompressIntake() {
            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            setMotionMagic(IntakeConstants.ANGLE_SLOW_MOTION_MAGIC_CONFIG);
            setLocation(IntakeState.UP_OFF.angleLocation);
        }

        @Override
        public void end(boolean interrupted) {
            setMotionMagic(IntakeConstants.ANGLE_FAST_MOTION_MAGIC_CONFIG);
        }
    }

    public class BounceIntake extends Command {

        private IntakeState state;
        private Timer timer;

        public BounceIntake() {
            addRequirements(Intake.this);
            timer = new Timer();
        }

        @Override
        public void initialize() {
            setLocation(IntakeState.BOUNCE_UP.angleLocation);
            state = IntakeState.BOUNCE_UP;
            timer.start();
            timer.reset();
        }

        @Override
        public void execute() {
            boolean alternate = isNear(state) || timer.get() > 1;
            System.out.println(alternate);
            System.out.println(timer.get());
            if (state == IntakeState.DOWN_OFF && alternate) {
                setLocation(IntakeState.BOUNCE_UP.angleLocation);
                state = IntakeState.BOUNCE_UP;
                timer.reset();
            } else if (state == IntakeState.BOUNCE_UP && alternate) {
                setLocation(IntakeState.DOWN_OFF.angleLocation);
                state = IntakeState.DOWN_OFF;
                timer.reset();
            }
        }

        @Override
        public void end(boolean interrupted) {
            setLocation(IntakeState.BOUNCE_UP.angleLocation);
        }

        private boolean isNear(IntakeState state) {
            return getPivotAngle().isNear(state.angleLocation, Degrees.of(1));
        }
    }
}
