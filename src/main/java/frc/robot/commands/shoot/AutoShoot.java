package frc.robot.commands.shoot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.AllianceUtil;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperSetpoint;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSetpoint;
import frc.robot.subsystems.shooter.FeederConstants.FeederSetpoint;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.ShotCalculatorStatus;

public class AutoShoot extends Shoot {

    // Subsystems
    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Hopper hopper;
    private final Intake intake;

    /** target pose2d (the hub based on alliance) */
    private Translation2d target;

    /** used to push adjusted robot pose to advantagescope robot sim */
    // private StructPublisher<Pose2d> adjustedRobotPosePublisher = NetworkTableInstance.getDefault()
    //     .getStructTopic("adjustedRobotPose", Pose2d.struct)
    //     .publish();

    public AutoShoot(Drivetrain drivetrain, Shooter shooter, Hopper hopper, Intake intake, boolean requireSubsystems) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.hopper = hopper;
        this.intake = intake;

        if (requireSubsystems) {
            addRequirements(drivetrain, shooter, hopper);
        }
    }

    @Override
    public void initialize() {
        this.target = FieldConstants.HUB_LOCATIONS.get(AllianceUtil.getAlliance());
        shooter.setFeederDutyCycle(FeederSetpoint.STOP);
        hopper.setDutyCycle(HopperSetpoint.STOP);
        CommandScheduler.getInstance().schedule(intake.new BounceIntake());
    }

    @Override
    public void execute() {
        var state = drivetrain.getState();
        var result = ShotCalculator.calculate(
            target,
            state.Pose.getTranslation(),
            ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation())
        );

        // check if in range, return if out of range
        if (result.status() == ShotCalculatorStatus.OUT_OF_RANGE) {
            CommandScheduler.getInstance().cancel(this);
            return;
        }

        // rotate the swerve to the desired angle
        rotateSwerve(drivetrain, result.yaw().baseUnitMagnitude());

        // set the desired hood angle
        shooter.setHoodTrajectoryPitch(result.pitch());
        shooter.setFlywheelVelocity(result.tangentialVelocity());

        // send tolerances to smart dashboard
        SmartDashboard.putBoolean("isAtPitch", shooter.isAtHoodPitch());
        SmartDashboard.putBoolean("isAtVelocity", shooter.isAtFlywheelVelocity());
        SmartDashboard.putBoolean("swerve isatangle", isAtDesiredRotation(state, result.yaw()));

        if (isReadyToShoot(drivetrain, shooter, result.yaw())) {
            // shoot the fuel if at the right pitch
            SmartDashboard.putString("Auto Aim Status", "Shooting");
            shooter.setFeederDutyCycle(FeederSetpoint.FEED);
            hopper.setDutyCycle(HopperSetpoint.FEED);
        } else {
            //otherwise just wait
            SmartDashboard.putString("Auto Aim Status", "Waiting");
        }

        if (RobotBase.isSimulation()) {
            simShoot(result, state);
        }

        SmartDashboard.putNumber("distance from hub", getDistanceFromHub());
        SmartDashboard.putNumber("autoaim desired pitch", result.pitch().in(Degrees));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.restHood();
        shooter.stopFlywheel();
        shooter.setFeederDutyCycle(FeederSetpoint.STOP);
        hopper.setDutyCycle(HopperSetpoint.STOP);
        CommandScheduler.getInstance().schedule(intake.new ChangeSetpoints(IntakeSetpoint.DOWN_OFF));
    }

    /** Units: meters */
    private double getDistanceFromHub() {
        return drivetrain.getState().Pose.getTranslation().getDistance(target);
    }
}
