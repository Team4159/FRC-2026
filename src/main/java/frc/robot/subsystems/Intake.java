package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.IntakeState;

public class Intake extends SubsystemBase {
    private final SparkFlex pivotMotor;
    private final SparkFlex spinMotor;
    private final CANcoder encoder;
    private double targetAngle;

    public Intake() {
        pivotMotor = new SparkFlex(Constants.IntakeConstants.kIntakePivotID, MotorType.kBrushless);
        spinMotor = new SparkFlex(Constants.IntakeConstants.kIntakeSpinID, MotorType.kBrushless);
        encoder = new CANcoder(Constants.IntakeConstants.kEncoderID);
        targetAngle = IntakeState.UP_OFF.angle;
    }

    public void setSpinSpeed(double speed) {
        spinMotor.set(speed);
    }

    public void setLocation(double angle) {
        targetAngle = angle;
    }

    @Override
    public void periodic(){
        //range [-0.5, 0.5)
        var currentAngle = encoder.getAbsolutePosition().getValueAsDouble();

        //convert to [0, 1)
        if(currentAngle < 0) currentAngle++;

        //PID
        double pidOutput = -Constants.IntakeConstants.intakePIDController.calculate(currentAngle, targetAngle);
        pivotMotor.set(pidOutput);
    }

    public class ChangeIntakeState extends Command {
        private IntakeState intakeState;

        public ChangeIntakeState(IntakeState intakeState) {
            this.intakeState = intakeState;
            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            setSpinSpeed(intakeState.spinPercentage);
            setLocation(intakeState.angle);
        }

        @Override
        public void end(boolean interrupt) {
            setSpinSpeed(0);
        }
    }
}
