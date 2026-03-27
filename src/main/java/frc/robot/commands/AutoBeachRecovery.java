package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.PoseUtil;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FieldConstants.TrenchZone;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class AutoBeachRecovery extends Command {
    
    public static enum BeachRecoveryMode {
        ZIG_ZAG,
        HOOK,
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

    private record PathTarget(APTarget target, Autopilot controller) {
        public PathTarget(APTarget target) {
            this(target, AutoConstants.kAutopilotCruiseController);
        }
    }

    private static final double kUnbeachTurnSpeed = 45.0;
    private static final double kUnbeachTranslationSpeed = DrivetrainConstants.kMaxTranslationSpeed / 2.0;
    private static final Distance kTrenchEntryDistance = Inches.of(60.0);

    private final Drivetrain drivetrain;
    private final Intake intake;

    private final Command shootCommand;

    private final BeachRecoveryMode beachRecoveryMode;
    private final BeachRecoverySide beachRecoverySide;
    private BeachRecoveryState beachRecoveryState;

    private PathTarget[] pathTargets = new PathTarget[0];
    private Autopilot pathController;
    private int pathProgress = 0;

    public AutoBeachRecovery(Drivetrain drivetrain, Intake intake, BeachRecoveryMode beachRecoveryMode, BeachRecoverySide beachRecoverySide, Command shootCommand) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.beachRecoveryMode = beachRecoveryMode;
        this.beachRecoverySide = beachRecoverySide;
        this.shootCommand = shootCommand;
    }

    @Override
    public void initialize() {
        beachRecoveryState = BeachRecoveryState.UNBEACH;
        pathTargets = getPathTargets(beachRecoveryMode, beachRecoverySide);
        pathProgress = 0;
        pathController = pathTargets[pathProgress].controller;
        CommandScheduler.getInstance().schedule(intake.new ChangeStates(IntakeState.DOWN_ON));
    }

    @Override
    public void execute() {
        if (beachRecoveryState == BeachRecoveryState.UNBEACH && !drivetrain.isSlipping()) {
            beachRecoveryState = BeachRecoveryState.RECOVER;
        }
        if (beachRecoveryState == BeachRecoveryState.RECOVER && pathProgress < pathTargets.length && pathController.atTarget(drivetrain.getState().Pose, pathTargets[pathProgress].target)) {
            pathProgress++;
            if (pathProgress < pathTargets.length) {
                pathController = pathTargets[pathProgress].controller;
            }
        }
        if (beachRecoveryState == BeachRecoveryState.RECOVER && pathProgress >= pathTargets.length) {
            beachRecoveryState = BeachRecoveryState.SHOOT;
            CommandScheduler.getInstance().schedule(intake.new ChangeStates(IntakeState.DOWN_OFF));
        }
        switch (beachRecoveryState) {
            case UNBEACH -> unbeach();
            case RECOVER -> recover();
            case SHOOT -> shoot();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setSpinSpeed(0.0);
        CommandScheduler.getInstance().cancel(shootCommand);
    }

    private void unbeach() {
        Angle driveAngle = Degrees.of(Math.abs((MathSharedStore.getTimestamp() * kUnbeachTurnSpeed % 360.0) - 180)).plus(Degrees.of(90.0));

        drivetrain.setControl(
          drivetrain.fieldCentricDrive
            .withVelocityX(kUnbeachTranslationSpeed * Math.cos(driveAngle.in(Radians)))
            .withVelocityY(kUnbeachTranslationSpeed * Math.sin(driveAngle.in(Radians)))
            .withRotationalRate(0.0)
        );
    }

    private void recover() {
        APResult result = pathController.calculate(drivetrain.getState().Pose, drivetrain.getState().Speeds, pathTargets[pathProgress].target);
        Rotation2d targetDirection = pathController == AutoConstants.kAutopilotAlignController ? result.targetAngle() : new Rotation2d(result.vx().baseUnitMagnitude(), result.vy().baseUnitMagnitude());
        drivetrain.setControl(
            drivetrain.trajectoryFacingAngleDrive
                .withVelocityX(result.vx())
                .withVelocityY(result.vy())
                .withTargetDirection(targetDirection)
        );
    }

    private void shoot() {
        CommandScheduler.getInstance().schedule(shootCommand);
    }

    private PathTarget[] getPathTargets(BeachRecoveryMode beachRecoveryMode, BeachRecoverySide beachRecoverySide) {
        // TODO: finish paths
        ArrayList<PathTarget> newPathTargets = new ArrayList<>();

        // targets are filtered to have no entry angle
        switch (beachRecoveryMode) {
            case ZIG_ZAG: {
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(300.0), Inches.of(79.0)));
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(280.0), Inches.of(40.0)));
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(260.0), Inches.of(130.0)));
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(230.0), Inches.of(130.0)));
                break;
            }
            case HOOK: {
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(300.0), Inches.of(79.0)));
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(270.0), Inches.of(130.0)));
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(230.0), Inches.of(130.0)));
                break;
            }
            case BLINE: {
                // nothing
                break;
            }
        }
        // return
        newPathTargets.add(new PathTarget(new APTarget(new Pose2d(TrenchZone.BLUE_RIGHT.x.plus(kTrenchEntryDistance), TrenchZone.BLUE_RIGHT.y, Rotation2d.kZero)), AutoConstants.kAutopilotCruiseController));
        newPathTargets.add(new PathTarget(new APTarget(new Pose2d(TrenchZone.BLUE_RIGHT.x.plus(kTrenchEntryDistance), TrenchZone.BLUE_RIGHT.y, Rotation2d.k180deg)), AutoConstants.kAutopilotAlignController));
        newPathTargets.add(new PathTarget(new APTarget(new Pose2d(TrenchZone.BLUE_RIGHT.x.minus(kTrenchEntryDistance), TrenchZone.BLUE_RIGHT.y, Rotation2d.k180deg)), AutoConstants.kAutopilotAlignController));
        newPathTargets.add(new PathTarget(new APTarget(new Pose2d(TrenchZone.BLUE_RIGHT.x.minus(kTrenchEntryDistance), Inches.of(50.0), Rotation2d.k180deg)), AutoConstants.kAutopilotAlignController));
        
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        for (int i = 0; i < newPathTargets.size(); i++) {
            Pose2d reference = newPathTargets.get(i).target.getReference();
            if (alliance == Alliance.Red) {
                reference = PoseUtil.flipPoseAlongMiddleXY(reference);
            }
            if (beachRecoverySide == BeachRecoverySide.LEFT) {
                reference = PoseUtil.flipPoseAlongMiddleY(reference);
            }
            newPathTargets.set(i, new PathTarget(newPathTargets.get(i).target.withReference(reference).withoutEntryAngle(), newPathTargets.get(i).controller));
        }

        return newPathTargets.toArray(PathTarget[]::new);
    }

    private void addIntakePointToPathTargets(ArrayList<PathTarget> pathTargets, Translation2d position) {
        pathTargets.add(new PathTarget(new APTarget(new Pose2d(position, Rotation2d.kZero)).withVelocity(DrivetrainConstants.kMaxTranslationSpeed * 0.5)));
    }
}
