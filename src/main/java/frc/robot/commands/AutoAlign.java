package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.FieldUtil;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.AlignConstants.TowerAlignGoal;
import frc.robot.Constants.OperatorConstants.DriveMode;
import frc.robot.subsystems.Drivetrain;

public class AutoAlign extends Command {
    private final Drivetrain drivetrain;
    private final TowerAlignGoal goal;
    private final BooleanSupplier robotRelativeModeSupplier;

    private APTarget currentTarget;
    private int progress;

    public AutoAlign(Drivetrain drivetrain, TowerAlignGoal goal, BooleanSupplier robotRelativeDriveSuppier) {
        this.drivetrain = drivetrain;
        this.goal = goal;
        this.robotRelativeModeSupplier = robotRelativeDriveSuppier;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        progress = 0;
        currentTarget = getNextTarget(progress);
    }

    @Override
    public void execute() {
        SwerveDriveState drivetrainState = drivetrain.getState();
        Pose2d drivetrainPose = drivetrainState.Pose;
        ChassisSpeeds drivetrainSpeeds = drivetrainState.Speeds;

        System.out.println(AlignConstants.kAlignController.atTarget(drivetrainPose, currentTarget));
        if (AlignConstants.kAlignController.atTarget(drivetrainPose, currentTarget)) {
            progress++;
            if (progress >= goal.targets.length) {
                return;
            }
            currentTarget = getNextTarget(progress);
        }

        APResult result = AlignConstants.kAlignController.calculate(drivetrainPose, drivetrainSpeeds,
                currentTarget);

        LinearVelocity velocityX = result.vx();
        LinearVelocity velocityY = result.vy();
        Rotation2d targetDirection = result.targetAngle();

        drivetrain.setControl(
                drivetrain.alignFacingAngleDrive
                        .withVelocityX(velocityX)
                        .withVelocityY(velocityY)
                        .withTargetDirection(targetDirection));
    }

    @Override
    public boolean isFinished() {
        return progress >= goal.targets.length;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            CommandScheduler.getInstance()
                    .schedule(drivetrain.new Drive(DriveMode.MANUAL_ALIGN, robotRelativeModeSupplier));
        }
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
