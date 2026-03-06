package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FieldUtil;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.AlignConstants.TowerAlignGoal;
import frc.robot.subsystems.Drivetrain;

public class AutoAlign extends Command {
    private final Drivetrain drivetrain;
    private final TowerAlignGoal goal;

    private APTarget currentTarget;
    private int progress;

    public AutoAlign(Drivetrain drivetrain, TowerAlignGoal goal) {
        this.drivetrain = drivetrain;
        this.goal = goal;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        progress = 0;
        currentTarget = getNextTarget(progress);
    }

    @Override
    public void execute() {
        boolean atTarget = AlignConstants.kAlignController.atTarget(drivetrain.getState().Pose, currentTarget);
        boolean targetIsIntermediate = (progress < goal.targets.length - 1);
        if (atTarget && targetIsIntermediate) {
            progress++;
            currentTarget = getNextTarget(progress);
        }

        if (atTarget && !targetIsIntermediate) {
            idle();
        } else {
            alignToTarget();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void idle() {
        drivetrain.setControl(drivetrain.idleDrive);
    }

    private void alignToTarget() {
        SwerveDriveState drivetrainState = drivetrain.getState();
        APResult result = AlignConstants.kAlignController.calculate(drivetrainState.Pose, drivetrainState.Speeds,
                currentTarget);

        LinearVelocity velocityX = result.vx();
        LinearVelocity velocityY = result.vy();
        Rotation2d targetDirection = result.targetAngle();

        drivetrain.setControl(
                drivetrain.trajectoryFacingAngleDrive
                        .withVelocityX(velocityX)
                        .withVelocityY(velocityY)
                        .withTargetDirection(targetDirection));
    }

    private APTarget getNextTarget(int progress) {
        APTarget nextTarget = goal.targets[progress];
        if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
            nextTarget = nextTarget.withReference(FieldUtil.flipPoseToOtherAlliance(nextTarget.getReference()));
            if (nextTarget.getEntryAngle().isPresent()) {
                nextTarget = nextTarget.withEntryAngle(nextTarget.getEntryAngle().get().plus(Rotation2d.k180deg));
            }
        }
        return nextTarget;
    }
}
