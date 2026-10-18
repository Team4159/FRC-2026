package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSetpoint;

public class Intake extends SubsystemBase {

    private final MotionMagicDutyCycle pivotMotionMagicDutyCycle;

    private double rollerDutyCycle;

    public Intake() {
        setPivotMotionMagicConfiguration(IntakeConstants.PIVOT_FAST_MOTION_MAGIC_CONFIGURATION);

        pivotMotionMagicDutyCycle = new MotionMagicDutyCycle(0.0);
        
        IntakeConstants.PIVOT_MOTOR.setPosition(0);

        setPivotAngle(IntakeSetpoint.DOWN_OFF.pivotAngle);
        rollerDutyCycle = 0.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/pivot angle (Degrees)", getPivotAngle().in(Degrees));
        SmartDashboard.putNumber("Intake/pivot angle (Rotations)", getPivotAngle().in(Rotations));
        SmartDashboard.putNumber("Intake/motionmagic setpoint angle (Degrees)", pivotMotionMagicDutyCycle.getPositionMeasure().in(Degrees));
        SmartDashboard.putNumber("Intake/motionmagic setpoint angle (Rotations)", pivotMotionMagicDutyCycle.getPositionMeasure().in(Rotations));
        SmartDashboard.putNumber(
            "Intake/pivot pid error",
            Units.rotationsToDegrees(IntakeConstants.PIVOT_MOTOR.getClosedLoopError().getValueAsDouble())
        );

        // if (getPivotAngle().in(Degrees) < 15.0) {
        //     setRollerDutyCycle(rollerDutyCycle);
        // } else {
        //     setRollerDutyCycle(0.0);
        // }
    }

    public void setRollerDutyCycle(double dutyCycle) {
        SmartDashboard.putNumber("Intake/roller duty cycle target", dutyCycle);
        IntakeConstants.ROLLER_MOTOR.set(dutyCycle);
    }

    public void setPivotMotionMagicConfiguration(MotionMagicConfigs motionMagicConfigs) {
        IntakeConstants.PIVOT_MOTOR.getConfigurator().apply(
            IntakeConstants.PIVOT_MOTOR_CONFIGURATION.withMotionMagic(motionMagicConfigs)
        );
    }

    public void setPivotAngle(Angle angle) {
        SmartDashboard.putString("Intake/pivot state", "angle " + angle.baseUnitMagnitude());
        SmartDashboard.putNumber("Intake/pivot target angle (Degrees)", angle.in(Degrees));
        SmartDashboard.putNumber("Intake/pivot target angle (Rotations)", angle.in(Rotations));
        IntakeConstants.PIVOT_MOTOR.setControl(pivotMotionMagicDutyCycle.withPosition(angle));
    }

    private Angle getPivotAngle() {
        return IntakeConstants.PIVOT_MOTOR.getPosition().getValue();
    }

    public void stopIntakePivotMotor() {
        SmartDashboard.putString("Intake/pivot state", "STOP");
        SmartDashboard.putNumber("Intake/pivot stop time", Timer.getFPGATimestamp());
        IntakeConstants.PIVOT_MOTOR.stopMotor();
    }

    public class ChangeSetpoints extends Command {

        private final IntakeSetpoint intakeSetpoint;
        private final Timer timer = new Timer();

        public ChangeSetpoints(IntakeSetpoint intakeSetpoint) {
            this.intakeSetpoint = intakeSetpoint;
            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            timer.restart();
            rollerDutyCycle = intakeSetpoint.rollerDutyCycle;
            setPivotAngle(intakeSetpoint.pivotAngle);
        }

        @Override
        public void execute() {
            if (isTimedOut()) {
                stopIntakePivotMotor();
            }
        }

        @Override
        public void end(boolean interrupt) {
            if (isTimedOut()) {
                stopIntakePivotMotor();
            } else {
                Intake.this.setRollerDutyCycle(IntakeSetpoint.UP_OFF.rollerDutyCycle);
            }
            rollerDutyCycle = 0.0;
        }

        private boolean isTimedOut() {
            return timer.hasElapsed(0.5);
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
            timer.restart();
        }

        @Override
        public void execute() {
            boolean alternate = isNear(intakeSetpoint) || timer.hasElapsed(1.0);
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
            // setPivotAngle(IntakeSetpoint.BOUNCE_UP.pivotAngle);
            stopIntakePivotMotor();
        }

        private boolean isNear(IntakeSetpoint intakeSetpoint) {
            return getPivotAngle().isNear(intakeSetpoint.pivotAngle, Degrees.of(5));
        }
    }
}
