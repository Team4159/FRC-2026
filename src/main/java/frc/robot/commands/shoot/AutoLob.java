package frc.robot.commands.shoot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.subsystems.shooter.FlywheelConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.AutoShootStatus;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterSetpoint;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.ShotCalculatorResult;
import frc.robot.subsystems.shooter.ShotCalculator.ShotCalculatorStatus;

public class AutoLob extends Shoot {

    //Subsystems
    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Hopper hopper;
    private final Intake intake;

    private final Timer timer = new Timer();

    /** used to push adjusted robot pose to advantagescope robot sim */
    // private StructPublisher<Pose2d> adjustedRobotPosePublisher = NetworkTableInstance.getDefault()
    //     .getStructTopic("adjustedRobotPose", Pose2d.struct)
    //     .publish();

    //advantagescope sim
    /** for sim testing to simulate loss of velocity */
    private double initializeTime;

    /**@param drivetrain the CommandSwerveDrivetrain
     * @param autonomousMode if set to true the actual robot swerve control will be disabled and the robot desired omega will be returned by the getDesiredOmega() function
     * it will also no longer require the drivetrain because a different command will be running for the auto path control to work
     * otherwise this constructor without the doublesuppliers will set the robot translation velocities to 0, it is designed to be used for auto
     */
    public AutoLob(Drivetrain drivetrain, Shooter shooter, Hopper hopper, Intake intake, boolean requireSubsystems) {
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
        CommandScheduler.getInstance().schedule(intake.new BounceIntake());
        initializeTime = MathSharedStore.getTimestamp();
        shooter.setFeederDutyCycle(FeederSetpoint.UNJAM);
        hopper.setDutyCycle(HopperSetpoint.STOP);
        shooter.setFlywheelMotorVelocity(ShooterSetpoint.LOB);
        timer.reset();
    }

    @Override
    public void execute() {
        //recalculate lob position
        Translation2d translation = drivetrain.getState().Pose.getTranslation();
        var target = FieldConstants.LOB_LOCATIONS.get(AllianceUtil.getAlliance())
            .stream()
            .min((t1, t2) -> {
                double distanceDifference = translation.getDistance(t1) - translation.getDistance(t2);
                return (int) (Math.ceil(Math.abs(distanceDifference)) * Math.signum(distanceDifference));
            })
            .get();

        //calculate desired pitch for hood angle
        double desiredHoodAngle = ShotCalculator.calculatePitch(
            translation.getDistance(target),
            RobotBase.isSimulation() ? getTangentialVelocity() : shooter.getFuelExitVelocity().baseUnitMagnitude(),
            FlywheelConstants.SHOOT_EFFICIENCY
        );
        double yaw = target.minus(translation).getAngle().getRadians();

        AutoShootStatus autoShootStatus = AutoShootStatus.WAITING;
        if (timer.hasElapsed(ShooterConstants.BACKWARDS_TIME) && isReadyToShoot(drivetrain, shooter, Radians.of(yaw))) {
            //shoot the fuel if at the right pitch
            autoShootStatus = AutoShootStatus.SHOOT;
            shooter.setFeederDutyCycle(FeederSetpoint.FEED);
            hopper.setDutyCycle(HopperSetpoint.FEED);
        }
        SmartDashboard.putString("Shooter/Auto Aim/Status", autoShootStatus.name());

        //rotate the swerve to the desired angle
        rotateSwerve(drivetrain, yaw);

        //set the desired hood angle
        shooter.setHoodTrajectoryPitch(Radians.of(desiredHoodAngle));

        //AdvantageScope fuel simulation
        if (RobotBase.isSimulation()) {
            simShoot(
                new ShotCalculatorResult(
                    ShotCalculatorStatus.SUCCESS,
                    MetersPerSecond.of(getTangentialVelocity()),
                    Radians.of(desiredHoodAngle),
                    Radians.of(yaw)
                ),
                drivetrain.getState()
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.restHood();
        //shooter.setSpeed(ShooterConstants.restingAngularVelocity);
        shooter.stopFlywheel();
        shooter.setFeederDutyCycle(FeederSetpoint.STOP);
        hopper.setDutyCycle(HopperSetpoint.STOP);
        CommandScheduler.getInstance().schedule(intake.new ChangeSetpoints(IntakeSetpoint.BOUNCE_UP));
    }

    /** @return currently returns theoretical max that declines at a rate of 0.1 m/s (to simulate shooter slowing down over time), but when implemented with shooter will return current launch velocity based on shooter angular velocity */
    private double getTangentialVelocity() {
        //currently returns theoretical max that declines at a rate of 0.1 m/s
        return Units.feetToMeters(29) - (MathSharedStore.getTimestamp() - initializeTime) * 0.1;
    }
}
