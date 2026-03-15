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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;

public class Intake extends SubsystemBase {
    private final TalonFX locationMotor;
    private final TalonFX spinMotor;
    private final CANcoder canCoder;

    private final MotionMagicVoltage intakeMotionMagicVoltage;
    // private final VelocityVoltage intakeVelocityVoltage; 

    public Intake() {
        locationMotor = new TalonFX(IntakeConstants.kAngleId);
        spinMotor = new TalonFX(IntakeConstants.kIntakeSpinId);
        canCoder = new CANcoder(IntakeConstants.kAngleEncoderId);

        System.out.println("cancoderconfig: " + IntakeConstants.canCoderConfig);

        canCoder.getConfigurator().apply(IntakeConstants.canCoderConfig);
        locationMotor.getConfigurator().apply(IntakeConstants.angleConfig);
        setMotionMagic(IntakeConstants.kFastMotionMagicConfig);

        intakeMotionMagicVoltage = new MotionMagicVoltage(0);
        setLocation(IntakeState.UP_OFF.rotationLocation);
        //intakeVelocityVoltage = new VelocityVoltage(0);

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(20)).withSupplyCurrentLimitEnable(true);
        spinMotor.getConfigurator().apply(currentLimits);
    }

    public void setMotionMagic(MotionMagicConfigs motionMagicConfigs){
        locationMotor.getConfigurator().apply(IntakeConstants.angleConfig.withMotionMagic(motionMagicConfigs));
    }

    public void setSpinSpeed(double speed) {
        spinMotor.set(speed);
    }

    public void setLocation(Angle angle) {
        locationMotor.setControl(intakeMotionMagicVoltage.withPosition(angle));
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
        public CompressIntake(){
            addRequirements(Intake.this);
        }

        @Override
        public void initialize(){
            setMotionMagic(IntakeConstants.kSlowMotionMagicConfig);
            setLocation(IntakeState.UP_OFF.rotationLocation);
        }

        @Override
        public void end(boolean interrupted){
            setMotionMagic(IntakeConstants.kFastMotionMagicConfig);
        }
    }

    public class BounceIntake extends Command{

        public BounceIntake(){
            addRequirements(Intake.this);
        }

        @Override
        public void initialize(){
            setLocation(IntakeState.BOUNCE_UP.rotationLocation);
        }

        @Override
        public void execute(){
            if(getPivotAngle().isNear( IntakeState.DOWN_OFF.rotationLocation, Degrees.of(5))){
                setLocation(IntakeState.BOUNCE_UP.rotationLocation);
            }
            if(getPivotAngle().isNear( IntakeState.BOUNCE_UP.rotationLocation, Degrees.of(5))){
                setLocation(IntakeState.DOWN_OFF.rotationLocation);
            }
        }

        @Override 
        public void end(boolean interrupted){
            setLocation(IntakeState.BOUNCE_UP.rotationLocation);
        }
    }
}
