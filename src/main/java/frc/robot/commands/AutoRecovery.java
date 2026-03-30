package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.DrivetrainConstants.kMaxTranslationSpeed;

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
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.FieldConstants.TrenchZone;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoRecovery extends Command {
    
    public static enum AutoRecoveryMode {
        ZIG_ZAG,
        HOOK,
        BLINE,
        SWEEP,
    }

    public static enum AutoRecoverySide {
        LEFT,
        RIGHT,
        MIDDLE,
    }

    private static enum AutoRecoveryState {
        UNBEACH,
        RECOVER,
    }

    private record PathTarget(APTarget target, Autopilot controller) {
        public PathTarget(APTarget target) {
            this(target, AutoConstants.kAutopilotCruiseController);
        }
    }

    private static final double kUnbeachTurnSpeed = 45.0;
    private static final double kUnbeachTranslationSpeed = DrivetrainConstants.kMaxTranslationSpeed / 2.0;
    private static final Distance kTrenchEntryDistance = Inches.of(60.0);
    private static final double kIntakeTranslationSpeed = DrivetrainConstants.kMaxTranslationSpeed * 0.33;

    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Intake intake;

    private final Translation2d finalTranslation;

    private final AutoRecoveryMode autoRecoveryMode;
    private final AutoRecoverySide autoRecoverySide;
    private AutoRecoveryState autoRecoveryState;

    private PathTarget[] pathTargets = new PathTarget[0];
    private Autopilot pathController;
    private int pathProgress = 0;

    private boolean started;
    private boolean finished;

    public AutoRecovery(Drivetrain drivetrain, Shooter shooter, Intake intake, AutoRecoveryMode autoRecoveryMode, AutoRecoverySide autoRecoverySide, Translation2d finalTranslation) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.intake = intake;
        this.autoRecoveryMode = autoRecoveryMode;
        this.autoRecoverySide = autoRecoverySide;
        this.finalTranslation = finalTranslation;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        if (!shouldRecover()) {
            started = false;
            finished = true;
            return;
        }
        started = true;
        finished = false;
        autoRecoveryState = AutoRecoveryState.RECOVER;
        pathTargets = getPathTargets(autoRecoveryMode, autoRecoverySide);
        pathProgress = 0;
        pathController = pathTargets[pathProgress].controller;
        shooter.restHood();
        CommandScheduler.getInstance().schedule(intake.new ChangeStates(IntakeState.DOWN_ON));
    }

    @Override
    public void execute() {
        if (finished) { return; }
        if (autoRecoveryState == AutoRecoveryState.UNBEACH && !drivetrain.isSlipping()) {
            autoRecoveryState = AutoRecoveryState.RECOVER;
        }
        if (autoRecoveryState == AutoRecoveryState.RECOVER && pathProgress < pathTargets.length && pathController.atTarget(drivetrain.getState().Pose, pathTargets[pathProgress].target)) {
            pathProgress++;
            if (pathProgress < pathTargets.length) {
                pathController = pathTargets[pathProgress].controller;
            } else {
                finished = true;
                return;
            }
        }
        switch (autoRecoveryState) {
            case UNBEACH -> unbeach();
            case RECOVER -> recover();
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        if (started && finished) {
            intake.setSpinSpeed(0.0);
        }
    }

    private boolean shouldRecover() {
        return !PoseUtil.isPoseInAllianceZone(DriverStation.getAlliance().orElse(Alliance.Blue),
                drivetrain.getState().Pose) && autoRecoverySide != AutoRecoverySide.MIDDLE;
    }

    private void unbeach() {
        // Angle driveAngle = Degrees.of(Math.abs((MathSharedStore.getTimestamp() * kUnbeachTurnSpeed % 360.0) - 180)).plus(Degrees.of(90.0));

        // if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        //     driveAngle = driveAngle.plus(Degrees.of(180.0));
        // }

        // drivetrain.setControl(
        //   drivetrain.fieldCentricDrive
        //     .withVelocityX(kUnbeachTranslationSpeed * Math.cos(driveAngle.in(Radians)))
        //     .withVelocityY(kUnbeachTranslationSpeed * Math.sin(driveAngle.in(Radians)))
        //     .withRotationalRate(0.0)
        // );
    }

    private void recover() {
        APResult result = pathController.calculate(drivetrain.getState().Pose, drivetrain.getState().Speeds, pathTargets[pathProgress].target);
        // Rotation2d targetDirection = pathController == AutoConstants.kAutopilotAlignController ? result.targetAngle() : new Rotation2d(result.vx().baseUnitMagnitude(), result.vy().baseUnitMagnitude());
        System.out.println("result: " + result);
        System.out.println("speeds: " + drivetrain.getState().Speeds);
        drivetrain.setControl(
            drivetrain.trajectoryFacingAngleDrive
                .withVelocityX(result.vx())
                .withVelocityY(result.vy())
                .withTargetDirection(result.targetAngle())
        );
    }

    private PathTarget[] getPathTargets(AutoRecoveryMode beachRecoveryMode, AutoRecoverySide beachRecoverySide) {
        ArrayList<PathTarget> newPathTargets = new ArrayList<>();

        // targets are filtered to have no entry angle
        switch (beachRecoveryMode) {
            case ZIG_ZAG: {
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(280.0), Inches.of(79.0)));
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(270.0), Inches.of(40.0)));
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(260.0), Inches.of(130.0)));
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(240.0), Inches.of(130.0)));
                break;
            }
            case HOOK: {
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(260.0), Inches.of(79.0)));
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(260.0), Inches.of(130.0)));
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(240.0), Inches.of(130.0)));
                break;
            }
            case BLINE: {
                // nothing
                break;
            }
            case SWEEP: {
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(270.0), Inches.of(40.0)));
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(270.0), Inches.of(150.0)));
                addIntakePointToPathTargets(newPathTargets, new Translation2d(Inches.of(240.0), Inches.of(150.0)));
                break;
            }
        }
        // newPathTargets.add(new PathTarget(new APTarget(new Pose2d(TrenchZone.BLUE_RIGHT.x.plus(kTrenchEntryDistance), TrenchZone.BLUE_RIGHT.y.plus(Inches.of((18.0))), Rotation2d.kZero)), AutoConstants.kAutopilotCruiseController));
        // newPathTargets.add(new PathTarget(new APTarget(new Pose2d(TrenchZone.BLUE_RIGHT.x.plus(kTrenchEntryDistance), TrenchZone.BLUE_RIGHT.y.plus(Inches.of((6.0))), Rotation2d.k180deg)), AutoConstants.kAutopilotAlignController));
        // newPathTargets.add(new PathTarget(new APTarget(new Pose2d(TrenchZone.BLUE_RIGHT.x.minus(kTrenchEntryDistance), TrenchZone.BLUE_RIGHT.y.plus(Inches.of((0.0))), Rotation2d.k180deg)), AutoConstants.kAutopilotAlignController));
        // newPathTargets.add(new PathTarget(new APTarget(new Pose2d(TrenchZone.BLUE_RIGHT.x.minus(kTrenchEntryDistance), Inches.of(50.0), Rotation2d.k180deg)), AutoConstants.kAutopilotAlignController));
        newPathTargets.add(new PathTarget(new APTarget(new Pose2d(TrenchZone.BLUE_RIGHT.x.plus(kTrenchEntryDistance), Inches.of(40.0), Rotation2d.kZero)), AutoConstants.kAutopilotCruiseController));
        newPathTargets.add(new PathTarget(new APTarget(new Pose2d(TrenchZone.BLUE_RIGHT.x.plus(kTrenchEntryDistance), TrenchZone.BLUE_RIGHT.y, Rotation2d.k180deg)), AutoConstants.kAutopilotAlignController));
        newPathTargets.add(new PathTarget(new APTarget(new Pose2d(TrenchZone.BLUE_RIGHT.x.minus(kTrenchEntryDistance), TrenchZone.BLUE_RIGHT.y, Rotation2d.k180deg)), AutoConstants.kAutopilotAlignController));

        
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        for (int i = 0; i < newPathTargets.size(); i++) {
            Pose2d reference = newPathTargets.get(i).target.getReference();
            if (alliance == Alliance.Red) {
                reference = PoseUtil.flipPoseAlongMiddleXY(reference);
            }
            if (beachRecoverySide == AutoRecoverySide.LEFT) {
                reference = PoseUtil.flipPoseAlongMiddleY(reference);
            }
            newPathTargets.set(i, new PathTarget(newPathTargets.get(i).target.withReference(reference).withoutEntryAngle().withVelocity(kMaxTranslationSpeed), newPathTargets.get(i).controller));
        }
        
        newPathTargets.add(new PathTarget(new APTarget(new Pose2d(finalTranslation, Rotation2d.k180deg)), AutoConstants.kAutopilotAlignController));

        return newPathTargets.toArray(PathTarget[]::new);
    }

    private void addIntakePointToPathTargets(ArrayList<PathTarget> pathTargets, Translation2d position) {
        // pathTargets.add(new PathTarget(new APTarget(new Pose2d(position, Rotation2d.kZero))));
        pathTargets.add(new PathTarget(new APTarget(new Pose2d(position, Rotation2d.kZero)).withVelocity(kIntakeTranslationSpeed)));
    }
}
