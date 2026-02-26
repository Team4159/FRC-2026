package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private interface VelocityControlRequest {
        ControlRequest withVelocity(AngularVelocity velocity);

        ControlRequest withVelocity(double velocity);
    }

    private enum ShooterState {
        STARTUP(new VelocityVoltage(0)),
        STEADY(new VelocityTorqueCurrentFOC(0));

        public final VelocityControlRequest controlRequest;

        private ShooterState(ControlRequest controlRequest) {
            this.controlRequest = (VelocityControlRequest) controlRequest;
        }
    }

    private static final AngularVelocity kSteadyTolerance = RotationsPerSecond.of(0.0);
    private static final AngularVelocity kRevVelocity = RotationsPerSecond.of(10.0);

    private final TalonFX leftTopMotor = new TalonFX(9);
    private final TalonFX leftBottomMotor = new TalonFX(10);
    private final TalonFX rightTopMotor = new TalonFX(11);
    private final TalonFX rightBottomMotor = new TalonFX(12);
    {
        TalonFXConfiguration allConfig = new TalonFXConfiguration();
        allConfig.Slot0.withKP(999999999999.0);
        allConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40.0)).withPeakReverseTorqueCurrent(Amps.of(0.0));
        allConfig.Voltage.withPeakForwardVoltage(Volts.of(12.0)).withPeakReverseVoltage(Volts.of(0.0));
        leftTopMotor.getConfigurator().apply(allConfig);
        leftBottomMotor.getConfigurator().apply(allConfig);
        rightTopMotor.getConfigurator().apply(allConfig);
        rightBottomMotor.getConfigurator().apply(allConfig);

        MotorOutputConfigs leftOutputConfigs = new MotorOutputConfigs();
        leftOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive);
        leftTopMotor.getConfigurator().apply(leftOutputConfigs);
        leftBottomMotor.getConfigurator().apply(leftOutputConfigs);

        MotorOutputConfigs rightOutputConfigs = new MotorOutputConfigs();
        rightOutputConfigs.withInverted(InvertedValue.Clockwise_Positive);
        rightTopMotor.getConfigurator().apply(rightOutputConfigs);
        rightBottomMotor.getConfigurator().apply(rightOutputConfigs);
    }

    private final TalonFX kickerMotor = new TalonFX(100);
    {
        kickerMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    }

    public Shooter() {
        stop();
    }

    public void setState(ShooterState state, AngularVelocity velocity) {
        var controlRequest = state.controlRequest.withVelocity(velocity);
        leftTopMotor.setControl(controlRequest);
        leftBottomMotor.setControl(controlRequest);
        rightTopMotor.setControl(controlRequest);
        rightBottomMotor.setControl(controlRequest);
    }

    public void stop() {
        leftTopMotor.stopMotor();
        leftBottomMotor.stopMotor();
        rightTopMotor.stopMotor();
        rightBottomMotor.stopMotor();
    }

    private final DutyCycleOut kickerControlRequest = new DutyCycleOut(0);

    public void setIntakeSpeed(double speed) {
        kickerControlRequest.withOutput(speed);
        kickerMotor.setControl(kickerControlRequest);
    }

    public class Shoot extends Command {
        private Supplier<AngularVelocity> velocitySupplier;
        private boolean steady;

        public Shoot(Supplier<AngularVelocity> velocitySupplier) {
            this.velocitySupplier = velocitySupplier;
            addRequirements(Shooter.this);
        }

        @Override
        public void initialize() {
            steady = false;
            setState(ShooterState.STARTUP, kRevVelocity);
        }

        @Override
        public void execute() {
            AngularVelocity motorVelocity = leftBottomMotor.getVelocity(true).getValue();
            AngularVelocity targetVelocity = velocitySupplier.get();
            if (!steady && motorVelocity.in(RPM) >= targetVelocity.in(RPM) - kSteadyTolerance.in(RPM)) {
                steady = true;
            }
            if (steady) {
                if (motorVelocity.in(RPM) > targetVelocity.in(RPM)) {
                    setState(ShooterState.STEADY, RPM.of(0.0));
                } else {
                    setState(ShooterState.STEADY, kRevVelocity);
                }
            }
        }

        @Override
        public void end(boolean interrupted) {
            stop();
        }
    }
}
