package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;//
import com.ctre.phoenix6.controls.PositionVoltage;//
import com.ctre.phoenix6.controls.PercentOutput;
import com.ctre.phoenix6.hardware.TalonFX;//

import edu.wpi.first.wpilibj2.command.Command;//
import edu.wpi.first.wpilibj2.command.SubsystemBase;//
import frc.robot.Constants.IntakeConstants;//
import frc.robot.Constants.IntakeConstants.IntakeState;//       

public class Intake extends SubsystemBase {
    private final TalonFX locationMotor, spinMotor;

    private final PositionVoltage intakePositionVoltage;
    private final PercentOutput spinPercentOutput;

    public Intake() {
        Slot0Configs locationConfig = new Slot0Configs();
        locationConfig.kP = IntakeConstants.kP;
        locationConfig.kI = IntakeConstants.kI;
        locationConfig.kD = IntakeConstants.kD;

        Slot0Configs spinConfig = new Slot0Configs();
        spinConfig.kP = IntakeConstants.kP;
        spinConfig.kI = IntakeConstants.kI;
        spinConfig.kD = IntakeConstants.kD;  

        locationMotor = new TalonFX(IntakeConstants.kIntakeLocationId);
        spinMotor = new TalonFX(IntakeConstants.kIntakeSpinId);

        locationMotor.getConfigurator().apply(locationConfig);
        //spin motor no PID right?

        intakePositionVoltage = new PositionVoltage(0);
        spinPercentOutput = new PercentOutput(0);
    }

    public void setSpinSpeed(double speed) {
        double speedWithGearRatio = speed * IntakeConstants.kSpinGearRatio;
        //spinMotor.setControl(intakeVelocityVoltage.withVelocity(speedWithGearRatio)); //I don't think I am properly accounting for gear ratio
        spinMotor.setControl(spinPercentOutput.withOutput(speedWithGearRatio));
        
    }

    public void setLocation(double angle) {
        double angleWithGearRatio = angle * IntakeConstants.kLocationGearRatio;
        locationMotor.setControl(intakePositionVoltage.withPosition(angleWithGearRatio)); //here too
    }

    public class IntakeCommand extends Command {
        private double angle;
        private double speed;
        private double currentLocation;

        public IntakeCommand(double angle, double speed) {
            this.angle = angle;
            this.speed = speed;

            currentLocation = 0;

            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            Intake.this.setLocation(angle);
            Intake.this.setSpinSpeed(speed);


            if (angle != currentLocation) { 
                Intake.this.setLocation(angle);
            }
            Intake.this.setSpinSpeed(speed);

            currentLocation = angle;
        }

        @Override
        public void end(boolean interrupt) {
            Intake.this.setLocation(0);
            Intake.this.setSpinSpeed(0);
            currentLocation = 0;
        }
    }

    public class ChangeStates extends Command {
        //private IntakeState intakeState;
        private final IntakeState intakeState;

        public ChangeStates(IntakeState state) {
            this.intakeState = state;

            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            Intake.this.setSpinSpeed(intakeState.spinSpeed);
        }

        @Override
        public void end(boolean interrupt) {
            if (interrupt) {
                Intake.this.setLocation(0);
            }
            
            Intake.this.setSpinSpeed(0);
        }
    }
}



