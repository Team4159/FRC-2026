package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;

public class Intake extends SubsystemBase {
    private final TalonFX locationMotor;
    private final TalonFX spinMotor;
    private final CANcoder canCoder;

    private final PIDController pidController = new PIDController(0.5, 0, 0);

    private double desiredAngle;

    private final PositionVoltage intakePositionVoltage;
    private final VelocityVoltage intakeVelocityVoltage; 

    public Intake() {
        desiredAngle = IntakeState.STOP.rotationLocation;
        Slot0Configs locationConfig = new Slot0Configs();
        TalonFXConfiguration locationFeedbackConfig = new TalonFXConfiguration();

        locationConfig.kP = IntakeConstants.kP;
        locationConfig.kI = IntakeConstants.kI;
        locationConfig.kD = IntakeConstants.kD;

        //external cancoder
        locationFeedbackConfig.Feedback.FeedbackRemoteSensorID = IntakeConstants.kCANCoderID;
        locationFeedbackConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        Slot0Configs spinConfig = new Slot0Configs();

        spinConfig.kP = IntakeConstants.kP;
        spinConfig.kI = IntakeConstants.kI;
        spinConfig.kD = IntakeConstants.kD;

        //CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

        locationMotor = new TalonFX(IntakeConstants.kIntakeLocationId);
        spinMotor = new TalonFX(IntakeConstants.kIntakeSpinId);
        canCoder = new CANcoder(IntakeConstants.kCANCoderID);

        
        locationMotor.getConfigurator().apply(locationFeedbackConfig);
        locationMotor.getConfigurator().apply(locationConfig);
        spinMotor.getConfigurator().apply(spinConfig);


        intakePositionVoltage = new PositionVoltage(0);
        intakeVelocityVoltage = new VelocityVoltage(0);

        setLocation(IntakeState.STOP.rotationLocation);
    }

    public void setSpinSpeed(double speed) {
        spinMotor.set(speed); //I don't think I am properly accounting for gear ratio
    }

    public void setLocation(double angle) {
        //locationMotor.setControl(intakePositionVoltage.withPosition(angle));
        //SmartDashboard.putNumber("", locationMotor.)
        //desiredAngle = angle; //here too
    }

    @Override
    public void periodic(){
        double currentAngle = locationMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("intake angle", currentAngle);
    }

    public class ChangeStates extends Command {
        private IntakeState state;

        public ChangeStates(IntakeState state) {
            this.state = state;
            

            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            //if (state.rotationLocation != currentLocation) { 
                Intake.this.setLocation(state.rotationLocation);
            //}
            Intake.this.setSpinSpeed(state.spinSpeed);

            //currentLocation = state.rotationLocation;
        }

        @Override
        public void end(boolean interrupt) {
            if (interrupt) {
                //Intake.this.setLocation(0);
            }
            
            Intake.this.setSpinSpeed(0);
        }
    }
}
