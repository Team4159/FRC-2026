package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.PoseUtil;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoBeachRecovery extends Command {
    
    public static enum BeachRecoveryMode {
        ZIG_ZAG_FULL,
        ZIG_ZAG_HALF,
        BLINE,
    }

    public static enum BeachRecoverySide {
        LEFT,
        RIGHT,
    }

    private static enum BeachRecoveryState {
        UNBEACH,
        RECOVER,
        SHOOT,
    }

    private static final double kUnbeachTurnSpeed = 45.0;
    private static final double kUnbeachTranslationSpeed = DrivetrainConstants.kMaxTranslationSpeed / 2.0;
    private static final Distance kTrenchEntryDistance = Inches.of(50.0);

    private final Drivetrain drivetrain;
    private final BeachRecoveryMode beachRecoveryMode;
    private final BeachRecoverySide beachRecoverySide;
    private BeachRecoveryState beachRecoveryState;

    private APTarget[] pathTargets = new APTarget[0]; 
    private int pathProgress = 0;

    public AutoBeachRecovery(Drivetrain drivetrain, BeachRecoveryMode beachRecoveryMode, BeachRecoverySide beachRecoverySide) {
        this.drivetrain = drivetrain;
        this.beachRecoveryMode = beachRecoveryMode;
        this.beachRecoverySide = beachRecoverySide;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        beachRecoveryState = BeachRecoveryState.UNBEACH;
        pathTargets = getPathTargets();
        pathProgress = 0;
    }

    @Override
    public void execute() {
        if (beachRecoveryState == BeachRecoveryState.UNBEACH && !drivetrain.isSlipping()) {
            beachRecoveryState = BeachRecoveryState.RECOVER;
        }
        if (beachRecoveryState == BeachRecoveryState.RECOVER && pathProgress >= pathTargets.length) {
            beachRecoveryState = BeachRecoveryState.SHOOT;
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
        APTarget target = pathTargets[pathProgress];
    }

    private void shoot() {

    }

    private APTarget[] getPathTargets() {
        // TODO: finish paths
        APTarget[] newPathTargets = switch(beachRecoveryMode) {
            case ZIG_ZAG_FULL -> new APTarget[0];
            case ZIG_ZAG_HALF -> new APTarget[0];
            case BLINE -> new APTarget[0];
        };

        for (int i = 0; i < newPathTargets.length; i++) {
            Pose2d reference = newPathTargets[i].getReference();
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                reference = PoseUtil.flipPoseAlongMiddleXY(reference);
            }
            if (beachRecoverySide == BeachRecoverySide.LEFT) {
                reference = PoseUtil.flipPoseAlongMiddleY(reference);
            }
            // TODO: decide whether or not to support entry angle
            newPathTargets[i] = newPathTargets[i].withReference(reference).withoutEntryAngle();
        }

        return newPathTargets;
    }
}
