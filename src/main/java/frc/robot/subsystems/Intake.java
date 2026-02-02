package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.IntakeState;

public class Intake extends SubsystemBase {
    private final TalonFX locationMotor;
    private final TalonFX spinMotor;

    private final PositionVoltage intakePositionVoltage;
    private final VelocityVoltage intakeVelocityVoltage; 

    public Intake() {
        Slot0Configs locationConfig = new Slot0Configs();

        locationConfig.kP = Constants.IntakeConstants.kP;
        locationConfig.kI = Constants.IntakeConstants.kI;
        locationConfig.kD = Constants.IntakeConstants.kD;

        Slot0Configs spinConfig = new Slot0Configs();

        spinConfig.kP = Constants.IntakeConstants.kP;
        spinConfig.kI = Constants.IntakeConstants.kI;
        spinConfig.kD = Constants.IntakeConstants.kD;  

        locationMotor = new TalonFX(Constants.IntakeConstants.kIntakeLocationId);
        spinMotor = new TalonFX(Constants.IntakeConstants.kIntakeSpinId);

        locationMotor.getConfigurator().apply(locationConfig);
        spinMotor.getConfigurator().apply(spinConfig);

        intakePositionVoltage = new PositionVoltage(0);
        intakeVelocityVoltage = new VelocityVoltage(0);
    }

    public void setSpinSpeed(double speed) {
        spinMotor.setControl(intakeVelocityVoltage.withVelocity(speed * Constants.IntakeConstants.kSpinGearRatio)); //I don't think I am properly accounting for gear ratio
    }

    public void setLocation(double angle) {
        locationMotor.setControl(intakePositionVoltage.withPosition(angle * Constants.IntakeConstants.kLocationGearRatio)); //here too
    }

    public class ChangeStates extends Command {
        private IntakeState state;

        private double currentLocation;

        public ChangeStates(IntakeState state) {
            this.state = state;

            currentLocation = 0;

            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            if (state.rotationLocation != currentLocation) { 
                Intake.this.setLocation(state.rotationLocation);
            }
            Intake.this.setSpinSpeed(state.spinSpeed);

            currentLocation = state.rotationLocation;
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
