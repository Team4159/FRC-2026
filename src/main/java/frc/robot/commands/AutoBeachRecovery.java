package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AutoBeachRecovery extends Command {
    public static enum BeachRecoveryMode {
        ZIG_ZAG,
        BLINE,
    }

    private static enum BeachRecoveryState {
        UNBEACH,
        RECOVER,
        SHOOT,
    }

    private final Drivetrain drivetrain;
    private final BeachRecoveryMode beachRecoveryMode;
    private BeachRecoveryState beachRecoveryState;

    public AutoBeachRecovery(Drivetrain drivetrain, BeachRecoveryMode beachRecoveryMode) {
        this.drivetrain = drivetrain;
        this.beachRecoveryMode = beachRecoveryMode;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        beachRecoveryState = BeachRecoveryState.UNBEACH;
    }

    @Override
    public void execute() {
        switch (beachRecoveryState) {
            case UNBEACH -> unbeach();
            case RECOVER -> recover();
            case SHOOT -> shoot();
        }
    }

    private void unbeach() {
        Angle driveAngle = Degrees.of(MathSharedStore.getTimestamp() * 10.0);

        drivetrain.setControl(
          drivetrain.fieldCentricDrive
            .withVelocityX()  
        );
    }

    private void recover() {

    }

    private void shoot() {

    }
}
