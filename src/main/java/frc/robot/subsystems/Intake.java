package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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

    private final PositionVoltage intakePositionVoltage;
    // private final VelocityVoltage intakeVelocityVoltage; 

    public Intake() {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfig.MagnetSensor.withMagnetOffset(IntakeConstants.kEncoderOffset);
    
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        angleConfig.Slot0.kP = IntakeConstants.kAngleP;  
        angleConfig.Slot0.kI = IntakeConstants.kAngleI;
        angleConfig.Slot0.kD = IntakeConstants.kAngleD;
        angleConfig.Slot0.kG = IntakeConstants.kAngleG;
        angleConfig.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
        //angleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        angleConfig.Feedback.FeedbackRemoteSensorID = IntakeConstants.kAngleEncoderId;
        angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        angleConfig.Feedback.SensorToMechanismRatio = IntakeConstants.kSensorToMechanismRatio;
        angleConfig.Feedback.RotorToSensorRatio = IntakeConstants.kMotorToSensorRatio;

        
        locationMotor = new TalonFX(IntakeConstants.kAngleId);
        spinMotor = new TalonFX(IntakeConstants.kIntakeSpinId);
        canCoder = new CANcoder(IntakeConstants.kAngleEncoderId);

        canCoder.getConfigurator().apply(canCoderConfig);
        locationMotor.getConfigurator().apply(angleConfig);
        //spinMotor.getConfigurator().apply(spinConfig);

        intakePositionVoltage = new PositionVoltage(0);
        setLocation(IntakeState.UP_OFF.rotationLocation);
        //intakeVelocityVoltage = new VelocityVoltage(0);
    }

    public void setSpinSpeed(double speed) {
        spinMotor.set(speed);
    }

    public void setLocation(Angle angle) {
        locationMotor.setControl(intakePositionVoltage.withPosition(angle));
    }

    private void setPivotPercentage(double percentage){
        locationMotor.set(percentage);
    }

    private Angle getPivotAngle(){
        return Rotations.of(locationMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("intake angle", getPivotAngle().in(Degrees));
        SmartDashboard.putNumber("intake pid error", Units.rotationsToDegrees(locationMotor.getClosedLoopError().getValueAsDouble()));
    }

    public class ChangeStates extends Command {
        private IntakeState state;

        public ChangeStates(IntakeState state) {
            this.state = state;
            addRequirements(Intake.this);
        }

        @Override
        public void initialize() {
            Intake.this.setSpinSpeed(state.spinSpeed);
            setLocation(state.rotationLocation);
        }

        @Override
        public void end(boolean interrupt) {
            Intake.this.setSpinSpeed(IntakeState.STOP.spinSpeed);
        }
    }
    
    public class CompressIntake extends Command{
        private Timer timer;

        public CompressIntake(){
            timer = new Timer();
        }

        @Override
        public void initialize(){
            timer.reset();
        }

        @Override
        public void execute(){
            double pidOutput = IntakeConstants.compressPID.calculate(getPivotAngle().in(Radians), IntakeState.UP_OFF.rotationLocation.in(Radians));
            setPivotPercentage(pidOutput);
        }
    }

    public class BounceIntake extends Command{

        public BounceIntake(){

        }

        @Override
        public void initialize(){
            setLocation(IntakeState.UP_OFF.rotationLocation);
        }

        @Override
        public void execute(){
            if(getPivotAngle().isNear( IntakeState.DOWN_OFF.rotationLocation, Degrees.of(5))){
                setLocation(IntakeState.UP_OFF.rotationLocation);
            }
            if(getPivotAngle().isNear( IntakeState.UP_OFF.rotationLocation, Degrees.of(5))){
                setLocation(IntakeState.DOWN_OFF.rotationLocation);
            }
        }
    }
}
