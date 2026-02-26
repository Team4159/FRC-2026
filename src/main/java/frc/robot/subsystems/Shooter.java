package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;

import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private enum ShooterState {
        STARTUP(new VelocityVoltage(0)::withVelocity),
        STEADY(new VelocityTorqueCurrentFOC(0)::withVelocity);

        public final Function<AngularVelocity, ControlRequest> controlRequestSupplier;

        private ShooterState(Function<AngularVelocity, ControlRequest> controlRequestSupplier) {
            this.controlRequestSupplier = controlRequestSupplier;
        }
    }

    // Top left motor is the leader motor
    private final TalonFX topLeftMotor = new TalonFX(kTopLeftMotorId);
    private final TalonFX bottomLeftMotor = new TalonFX(kBottomLeftMotorId);
    private final TalonFX topRightMotor = new TalonFX(kTopRightMotorId);
    private final TalonFX bottomRightMotor = new TalonFX(kBottomRightMotorId);
    private final TalonFX leaderMotor = topLeftMotor;
    {
        TalonFXConfiguration allConfig = new TalonFXConfiguration();
        allConfig.Slot0.withKP(999999999999.0);
        allConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40.0)).withPeakReverseTorqueCurrent(Amps.of(0.0));
        allConfig.Voltage.withPeakForwardVoltage(Volts.of(12.0)).withPeakReverseVoltage(Volts.of(0.0));
        topLeftMotor.getConfigurator().apply(allConfig);
        bottomLeftMotor.getConfigurator().apply(allConfig);
        topRightMotor.getConfigurator().apply(allConfig);
        bottomRightMotor.getConfigurator().apply(allConfig);

        MotorOutputConfigs leftOutputConfigs = new MotorOutputConfigs();
        leftOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive);
        topLeftMotor.getConfigurator().apply(leftOutputConfigs);
        bottomLeftMotor.getConfigurator().apply(leftOutputConfigs);

        MotorOutputConfigs rightOutputConfigs = new MotorOutputConfigs();
        rightOutputConfigs.withInverted(InvertedValue.Clockwise_Positive);
        topRightMotor.getConfigurator().apply(rightOutputConfigs);
        bottomRightMotor.getConfigurator().apply(rightOutputConfigs);

        bottomLeftMotor.setControl(new StrictFollower(9));
        topRightMotor.setControl(new StrictFollower(9));
        bottomRightMotor.setControl(new StrictFollower(9));
    }

    private final TalonFX feederMotor = new TalonFX(kFeederMotorId);
    {
        feederMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velocity", bottomLeftMotor.getVelocity(true).getValue().in(RPM));
    }

    public Shooter() {
        stop();
    }

    public void setState(ShooterState state, AngularVelocity velocity) {
        SmartDashboard.putString("Shooter State", state.name());
        var controlRequest = state.controlRequestSupplier.apply(velocity);
        leaderMotor.setControl(controlRequest);
    }

    public void stop() {
        leaderMotor.stopMotor();
    }

    private final DutyCycleOut kickerControlRequest = new DutyCycleOut(0);

    public void setKickerSpeed(double speed) {
        kickerControlRequest.withOutput(speed);
        feederMotor.setControl(kickerControlRequest);
    }

    public class Shoot extends Command {
        private final Supplier<AngularVelocity> velocitySupplier;
        private boolean steady;

        public Shoot(Supplier<AngularVelocity> velocitySupplier) {
            this.velocitySupplier = velocitySupplier;
            addRequirements(Shooter.this);
        }

        @Override
        public void initialize() {
            steady = false;
        }

        @Override
        public void execute() {
            AngularVelocity motorVelocity = leaderMotor.getVelocity(true).getValue();
            AngularVelocity targetVelocity = velocitySupplier.get();
            if (motorVelocity.compareTo(targetVelocity) >= 0) {
                steady = true;
                stop();
            } else if (steady) {
                setState(ShooterState.STEADY, targetVelocity);
            } else {
                setState(ShooterState.STARTUP, targetVelocity);
            }
        }

        @Override
        public void end(boolean interrupted) {
            stop();
        }
    }
}
