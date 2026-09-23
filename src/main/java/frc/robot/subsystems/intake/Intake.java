package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

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
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;

public class Intake extends SubsystemBase {

    private final TalonFX pivotMotor;
    private final TalonFX rollerMotor;
    private final CANcoder pivotEncoder;

    private final MotionMagicVoltage pivotMotionMagicVoltage;

    private double rollerDutyCycle;

    public Intake() {
        pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID);
        rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);
        pivotEncoder = new CANcoder(IntakeConstants.PIVOT_ENCODER_ID);

        pivotEncoder.getConfigurator().apply(IntakeConstants.PIVOT_ENCODER_CONFIGURATION);
        setPivotMotionMagicConfiguration(IntakeConstants.PIVOT_FAST_MOTION_MAGIC_CONFIGURATION);
        pivotMotor.getConfigurator().apply(IntakeConstants.PIVOT_MOTOR_CONFIGURATION);
        rollerMotor.getConfigurator().apply(IntakeConstants.ROLLER_MOTOR_CONFIGURATION);

        pivotMotionMagicVoltage = new MotionMagicVoltage(0.0);
        setPivotAngle(IntakeState.DOWN_OFF.pivotAngle);
        rollerDutyCycle = 0.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake angle", getPivotAngle().in(Degrees));
        SmartDashboard.putNumber(
            "intake pid error",
            Units.rotationsToDegrees(pivotMotor.getClosedLoopError().getValueAsDouble())
        );

        if (getPivotAngle().in(Degrees) < 15.0) {
            setRollerDutyCycle(rollerDutyCycle);
        } else {
            setRollerDutyCycle(0.0);
        }
    }

    public void setPivotMotionMagicConfiguration(MotionMagicConfigs motionMagicConfigs) {
        pivotMotor
            .getConfigurator()
            .apply(IntakeConstants.PIVOT_MOTOR_CONFIGURATION.withMotionMagic(motionMagicConfigs));
    }

    public void setPivotAngle(Angle angle) {
        pivotMotor.setControl(pivotMotionMagicVoltage.withPosition(angle));
    }

    private Angle getPivotAngle() {
        return Rotations.of(pivotMotor.getPosition().getValueAsDouble());
    }

    public void setRollerDutyCycle(double dutyCycle) {
        rollerMotor.set(dutyCycle);
    }

    public class ChangeStates extends Command {

        private final IntakeState state;

        public ChangeStates(IntakeState state) {
            this.state = state;
            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            rollerDutyCycle = state.rollerDutyCycle;
            setPivotAngle(state.pivotAngle);
        }

        @Override
        public void end(boolean interrupt) {
            Intake.this.setRollerDutyCycle(IntakeState.UP_OFF.rollerDutyCycle);
            rollerDutyCycle = 0.0;
        }
    }

    public class CompressIntake extends Command {

        public CompressIntake() {
            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            setPivotMotionMagicConfiguration(IntakeConstants.PIVOT_SLOW_MOTION_MAGIC_CONFIGURATION);
            setPivotAngle(IntakeState.UP_OFF.pivotAngle);
        }

        @Override
        public void end(boolean interrupted) {
            setPivotMotionMagicConfiguration(IntakeConstants.PIVOT_FAST_MOTION_MAGIC_CONFIGURATION);
        }
    }

    public class BounceIntake extends Command {

        private final Timer timer;

        private IntakeState state;

        public BounceIntake() {
            addRequirements(Intake.this);
            timer = new Timer();
        }

        @Override
        public void initialize() {
            setPivotAngle(IntakeState.BOUNCE_UP.pivotAngle);
            state = IntakeState.BOUNCE_UP;
            timer.start();
            timer.reset();
        }

        @Override
        public void execute() {
            boolean alternate = isNear(state) || timer.get() > 1;
            // System.out.println(alternate);
            // System.out.println(timer.get());
            if (state == IntakeState.DOWN_OFF && alternate) {
                setPivotAngle(IntakeState.BOUNCE_UP.pivotAngle);
                state = IntakeState.BOUNCE_UP;
                timer.reset();
            } else if (state == IntakeState.BOUNCE_UP && alternate) {
                setPivotAngle(IntakeState.DOWN_OFF.pivotAngle);
                state = IntakeState.DOWN_OFF;
                timer.reset();
            }
        }

        @Override
        public void end(boolean interrupted) {
            setPivotAngle(IntakeState.BOUNCE_UP.pivotAngle);
        }

        private boolean isNear(IntakeState state) {
            return getPivotAngle().isNear(state.pivotAngle, Degrees.of(5));
        }
    }
}
