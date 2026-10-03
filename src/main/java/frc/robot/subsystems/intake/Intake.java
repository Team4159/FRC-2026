package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSetpoint;

public class Intake extends SubsystemBase {

    private final MotionMagicVoltage pivotMotionMagicVoltage;

    private double rollerDutyCycle;

    public Intake() {
        setPivotMotionMagicConfiguration(IntakeConstants.PIVOT_FAST_MOTION_MAGIC_CONFIGURATION);

        pivotMotionMagicVoltage = new MotionMagicVoltage(0.0);
        setPivotAngle(IntakeSetpoint.DOWN_OFF.pivotAngle);
        rollerDutyCycle = 0.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/angle", getPivotAngle().in(Degrees));
        SmartDashboard.putNumber(
            "Intake/pid error",
            Units.rotationsToDegrees(IntakeConstants.PIVOT_MOTOR.getClosedLoopError().getValueAsDouble())
        );

        if (getPivotAngle().in(Degrees) < 15.0) {
            setRollerDutyCycle(rollerDutyCycle);
        } else {
            setRollerDutyCycle(0.0);
        }
    }

    public void setPivotMotionMagicConfiguration(MotionMagicConfigs motionMagicConfigs) {
        IntakeConstants.PIVOT_MOTOR.getConfigurator().apply(
            IntakeConstants.PIVOT_MOTOR_CONFIGURATION.withMotionMagic(motionMagicConfigs)
        );
    }

    public void setPivotAngle(Angle angle) {
        IntakeConstants.PIVOT_MOTOR.setControl(pivotMotionMagicVoltage.withPosition(angle));
    }

    private Angle getPivotAngle() {
        return Rotations.of(IntakeConstants.PIVOT_MOTOR.getPosition().getValueAsDouble());
    }

    public void setRollerDutyCycle(double dutyCycle) {
        IntakeConstants.ROLLER_MOTOR.set(dutyCycle);
    }

    public class ChangeSetpoints extends Command {

        private final IntakeSetpoint intakeSetpoint;

        public ChangeSetpoints(IntakeSetpoint intakeSetpoint) {
            this.intakeSetpoint = intakeSetpoint;
            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            rollerDutyCycle = intakeSetpoint.rollerDutyCycle;
            setPivotAngle(intakeSetpoint.pivotAngle);
        }

        @Override
        public void end(boolean interrupt) {
            Intake.this.setRollerDutyCycle(IntakeSetpoint.UP_OFF.rollerDutyCycle);
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
            setPivotAngle(IntakeSetpoint.UP_OFF.pivotAngle);
        }

        @Override
        public void end(boolean interrupted) {
            setPivotMotionMagicConfiguration(IntakeConstants.PIVOT_FAST_MOTION_MAGIC_CONFIGURATION);
        }
    }

    public class BounceIntake extends Command {

        private final Timer timer;

        private IntakeSetpoint intakeSetpoint;

        public BounceIntake() {
            addRequirements(Intake.this);
            timer = new Timer();
        }

        @Override
        public void initialize() {
            setPivotAngle(IntakeSetpoint.BOUNCE_UP.pivotAngle);
            intakeSetpoint = IntakeSetpoint.BOUNCE_UP;
            timer.start();
            timer.reset();
        }

        @Override
        public void execute() {
            boolean alternate = isNear(intakeSetpoint) || timer.get() > 1.0;
            // System.out.println(alternate);
            // System.out.println(timer.get());
            if (intakeSetpoint == IntakeSetpoint.DOWN_OFF && alternate) {
                setPivotAngle(IntakeSetpoint.BOUNCE_UP.pivotAngle);
                intakeSetpoint = IntakeSetpoint.BOUNCE_UP;
                timer.reset();
            } else if (intakeSetpoint == IntakeSetpoint.BOUNCE_UP && alternate) {
                setPivotAngle(IntakeSetpoint.DOWN_OFF.pivotAngle);
                intakeSetpoint = IntakeSetpoint.DOWN_OFF;
                timer.reset();
            }
        }

        @Override
        public void end(boolean interrupted) {
            setPivotAngle(IntakeSetpoint.BOUNCE_UP.pivotAngle);
        }

        private boolean isNear(IntakeSetpoint intakeSetpoint) {
            return getPivotAngle().isNear(intakeSetpoint.pivotAngle, Degrees.of(5));
        }
    }
}
