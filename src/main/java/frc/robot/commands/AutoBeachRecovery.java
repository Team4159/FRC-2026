package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
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

    private static final double kUnbeachTurnSpeed = 45.0;
    private static final double kUnbeachTranslationSpeed = DrivetrainConstants.kMaxTranslationSpeed / 2.0;

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
        if (beachRecoveryState == BeachRecoveryState.UNBEACH && !drivetrain.isSlipping()) {
            beachRecoveryState = BeachRecoveryState.RECOVER;
        }
        switch (beachRecoveryState) {
            case UNBEACH -> unbeach();
            case RECOVER -> recover();
            case SHOOT -> shoot();
        }
    }

    private void unbeach() {
        Angle driveAngle = Degrees.of(Math.abs((MathSharedStore.getTimestamp() * kUnbeachTurnSpeed % 360.0) - 180)).plus(Degrees.of(90.0));

        drivetrain.setControl(
          drivetrain.fieldCentricDrive
            .withVelocityX(kUnbeachTranslationSpeed * Math.cos(driveAngle.in(Radians)))
            .withVelocityY(kUnbeachTranslationSpeed * Math.sin(driveAngle.in(Radians)))  
        );
    }

    private void recover() {

    }

    private void shoot() {

    }
}
