package frc.robot.subsystems.shooter;

public class FeederConstants {

    public static final int FEEDER_MOTOR_ID = 20;

    public static enum FeederState {
        FEED(1.0),
        UNJAM(-1.0),
        STOP(0.0);

        public final double dutyCycle;

        private FeederState(double dutyCycle) {
            this.dutyCycle = dutyCycle;
        }
    }
}
