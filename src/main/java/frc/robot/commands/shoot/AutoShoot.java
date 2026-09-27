package frc.robot.commands.shoot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.AllianceUtil;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.shooter.FeederConstants.FeederState;
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
        shooter.setFeederDutyCycle(FeederState.STOP.dutyCycle);
        hopper.setDutyCycle(HopperState.STOP.dutyCycle);
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
        rotateSwerve(result.yaw().baseUnitMagnitude());

        // set the desired hood angle
        shooter.setHoodTrajectoryPitch(result.pitch());
        shooter.setFlywheelVelocity(result.tangentialVelocity());

        // send tolerances to smart dashboard
        SmartDashboard.putBoolean("isAtPitch", shooter.isAtHoodPitch());
        SmartDashboard.putBoolean("isAtVelocity", shooter.isAtFlywheelVelocity());
        SmartDashboard.putBoolean("swerve isatangle", isAtDesiredRotation(result.yaw()));

        if (shooter.isAtHoodPitch() && shooter.isAtFlywheelVelocity() && isAtDesiredRotation(result.yaw())) {
            // shoot the fuel if at the right pitch
            SmartDashboard.putString("Auto Aim Status", "Shooting");
            shooter.setFeederDutyCycle(FeederState.FEED.dutyCycle);
            hopper.setDutyCycle(HopperState.FEED.dutyCycle);
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
        shooter.setFeederDutyCycle(FeederState.STOP.dutyCycle);
        hopper.setDutyCycle(HopperState.STOP.dutyCycle);
        CommandScheduler.getInstance().schedule(intake.new ChangeStates(IntakeState.DOWN_OFF));
    }

    /**
     * @param desiredAngle the desired field relative angle for the drivetrain
     *                     This also translates the robot using the getInputX() and
     *                     getInputY() functions in the Drivetrain class
     */
    private void rotateSwerve(double desiredAngle) {
        boolean aimFinished;
        if (drivetrain.getInputTranslation(true).getNorm() == 0.0) {
            boolean aimingAtHub = isAtDesiredRotation(Radians.of(desiredAngle));
            boolean robotIsStill = Math.toDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond) <= 1;
            aimFinished = aimingAtHub && robotIsStill;
        } else {
            aimFinished = false;
        }

        double omega = DrivetrainConstants.AUTO_SHOOT_ROTATION_CONTROLLER.calculate(
            drivetrain.getState().Pose.getRotation().getRadians(),
            desiredAngle,
            Timer.getFPGATimestamp()
        );

        if (aimFinished) {
            drivetrain.setControl(drivetrain.brakeDrive);
            return;
        }
        // PID controller to calculate omega
        // set ChassisSpeeds
        // System.out.println("drivetrain getInputX: " + drivetrain.getInputX(true));
        // System.out.println("drivetrain getInputY: " + drivetrain.getInputY(true));
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            drivetrain.getInputX(true) * DrivetrainConstants.AUTO_SHOOT_INPUT_MULTIPLIER,
            drivetrain.getInputY(true) * DrivetrainConstants.AUTO_SHOOT_INPUT_MULTIPLIER,
            omega
        );

        drivetrain.setControl(
            drivetrain.fieldCentricDrive
                .withVelocityX(chassisSpeeds.vxMetersPerSecond)
                .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(omega)
        );
    }

    private boolean isAtDesiredRotation(Angle angle) {
        return drivetrain
            .getState()
            .Pose.getRotation()
            .getMeasure()
            .isNear(angle, DrivetrainConstants.AUTO_SHOOT_TOLERANCE);
    }

    /** Units: meters */
    private double getDistanceFromHub() {
        return drivetrain.getState().Pose.getTranslation().getDistance(target);
    }
}
