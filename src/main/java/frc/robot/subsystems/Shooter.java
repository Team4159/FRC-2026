package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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

    private static final AngularVelocity kSteadyTolerance = RPM.of(25);
    private static final AngularVelocity kRevVelocity = RPM.of(10.0);

    private final TalonFX motor = new TalonFX(9);
    {
        motor.getConfigurator().apply(new Slot0Configs().withKP(999999999999.0));
        motor.getConfigurator().apply(new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(Amps.of(40.0))
                .withPeakReverseTorqueCurrent(Amps.of(0.0)));
        motor.getConfigurator().apply(
                new VoltageConfigs().withPeakForwardVoltage(Volts.of(12.0)).withPeakReverseVoltage(Volts.of(0.0)));
    }

    public Shooter() {
        stop();
    }

    public void setState(ShooterState state, AngularVelocity velocity) {
        motor.setControl(state.controlRequest.withVelocity(velocity));
    }

    public void stop() {
        motor.stopMotor();
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
            AngularVelocity motorVelocity = motor.getVelocity(true).getValue();
            AngularVelocity targetVelocity = velocitySupplier.get();
            if (!steady && Math.abs(motorVelocity.in(RPM) - targetVelocity.in(RPM)) <= kSteadyTolerance.in(RPM)) {
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
