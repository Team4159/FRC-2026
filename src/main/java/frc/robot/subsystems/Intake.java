package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.IntakeLocationState;
import frc.robot.Constants.IntakeConstants.IntakeSpinState;

public class Intake extends SubsystemBase {
    private TalonFX locationMotor;
    private TalonFX spinMotor;

    private final PositionVoltage intakePositionVoltage;
    private final VelocityVoltage intakeVelocityVoltage;

    private static final double locationGearRatio = 1.0/64.0; //placeholder
    private static final double spinGearRatio = 1.0/28.0; //placeholder

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
        spinMotor.setControl(intakeVelocityVoltage.withVelocity(speed / spinGearRatio)); //I don't think I am properly accounting for gear ratio
    }

    public void setLocation(double angle) {
        locationMotor.setControl(intakePositionVoltage.withPosition(angle / locationGearRatio)); //here too
    }

    public class ChangeStates extends Command {
        private IntakeLocationState location;
        private IntakeSpinState spinSpeed;

        private double currentLocation;

        public ChangeStates(IntakeLocationState location, IntakeSpinState spinSpeed) {
            this.location = location;
            this.spinSpeed = spinSpeed;
            currentLocation = 0;

            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            if (location.rotationLocation != currentLocation) { 
                Intake.this.setLocation(location.rotationLocation);
            }
            Intake.this.setSpinSpeed(spinSpeed.spinSpeed);

            currentLocation = location.rotationLocation;
        }

        @Override
        public void end(boolean interrupt) {
            Intake.this.setSpinSpeed(0);
        }
    }
}
