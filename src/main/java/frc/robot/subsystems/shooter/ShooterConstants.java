package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;

public class ShooterConstants {

    public static enum ShooterSetpoint {
        RESTING(RPM.of(0.0)),
        REV(RPM.of(1000.0)),
        LOB(RPM.of(2000.0)),
        FROM_HUB(RPM.of(2500.0), Degrees.of(75.0)),
        FROM_TOWER(RPM.of(3000.0), Degrees.of(70.0));

        public final AngularVelocity angularVelocity;
        public final Optional<Angle> pitch;

        private ShooterSetpoint(AngularVelocity angularVelocity, Angle pitch) {
            this.angularVelocity = angularVelocity;
            this.pitch = Optional.of(pitch);
        }

        private ShooterSetpoint(AngularVelocity angularVelocity) {
            this.angularVelocity = angularVelocity;
            this.pitch = Optional.empty();
        }
    }

    public static enum AutoShootStatus {
        SHOOT,
        OUT_OF_RANGE,
        WAITING,
    }

    public static final Time BACKWARDS_TIME = Seconds.of(0.05);
}
