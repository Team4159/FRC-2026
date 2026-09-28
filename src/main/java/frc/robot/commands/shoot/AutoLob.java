package frc.robot.commands.shoot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.AllianceUtil;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PhysicsConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.shooter.FeederConstants.FeederState;
import frc.robot.subsystems.shooter.JoeLookupTableConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.AutoShootStatus;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterSetpoint;
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
        double desiredHoodAngle = getDesiredHoodPitch(translation.getDistance(target));
        double yaw = target.minus(translation).getAngle().getRadians();

        AutoShootStatus autoShootStatus = AutoShootStatus.WAITING;
        if (!timer.hasElapsed(ShooterConstants.BACKWARDS_TIME)) {
            //run neck backwards if at the beginning
            shooter.setFeederDutyCycle(FeederState.UNJAM);
            hopper.setDutyCycle(HopperState.STOP);
        } else if (shooter.isAtHoodPitch() && shooter.isAtFlywheelVelocity() && isAtDesiredRotation(Radians.of(yaw))) {
            //shoot the fuel if at the right pitch
            autoShootStatus = AutoShootStatus.SHOOT;
            shooter.setFeederDutyCycle(FeederState.FEED);
            hopper.setDutyCycle(HopperState.FEED);
        }
        SmartDashboard.putString("Auto Aim Status", autoShootStatus.name());

        //rotate the swerve to the desired angle
        rotateSwerve(yaw);

        //set the desired hood angle
        shooter.setHoodTrajectoryPitch(Radians.of(desiredHoodAngle));

        SmartDashboard.putBoolean("isAtPitch", shooter.isAtHoodPitch());
        SmartDashboard.putBoolean("isAtVelocity", shooter.isAtFlywheelVelocity());
        SmartDashboard.putBoolean("swerve isatangle", isAtDesiredRotation(Radians.of(yaw)));

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

    /** @param yaw the desired field relative angle for the drivetrain
     * This also translates the robot using the getInputX() and getInputY() functions in the Drivetrain class
     */
    private void rotateSwerve(double yaw) {
        //PID controller to calculate omega
        double omega = DrivetrainConstants.AUTO_SHOOT_ROTATION_CONTROLLER.calculate(
            drivetrain.getState().Pose.getRotation().getRadians(),
            yaw,
            Timer.getFPGATimestamp()
        );
        //set ChassisSpeeds
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            drivetrain.getInputVelocityX(true) * DrivetrainConstants.AUTO_SHOOT_INPUT_MULTIPLIER,
            drivetrain.getInputVelocityY(true) * DrivetrainConstants.AUTO_SHOOT_INPUT_MULTIPLIER,
            omega
        );

        drivetrain.setControl(
            drivetrain.fieldCentricDrive
                .withVelocityX(chassisSpeeds.vxMetersPerSecond)
                .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(omega)
        );
    }

    /** @return the desired pitch for the hood based on the adjusted robot position */
    private double getDesiredHoodPitch(double distance) {
        // distance from robot to target
        double launchVelocity;
        if (RobotBase.isSimulation()) {
            launchVelocity = getTangentialVelocity();
        } else {
            launchVelocity = shooter.getFuelExitVelocity().baseUnitMagnitude();
        }
        double desiredPitch = Math.atan(
            (Math.pow(launchVelocity, 2) +
                Math.sqrt(
                    Math.pow(launchVelocity, 4) -
                        Math.pow(PhysicsConstants.GRAVITY * distance, 2) -
                        2 *
                            PhysicsConstants.GRAVITY *
                            JoeLookupTableConstants.TARGET_HEIGHT *
                            Math.pow(launchVelocity, 2)
                )) /
                (PhysicsConstants.GRAVITY * distance)
        );

        if (Double.isNaN(desiredPitch)) {
            //equation can only return angles from 45-90 deg (in radians of course), anything lower than that will be NaN
            //the minimum possible hood angle on the physical shooter is 45, so no additional calculation is needed, just set to 45
            desiredPitch = Units.degreesToRadians(45);
        }
        // if(desiredPitch > ShooterConstants.maxPitch){
        //     desiredPitch = ShooterConstants.maxPitch;
        // }
        SmartDashboard.putNumber("autoaim desired pitch", Units.radiansToDegrees(desiredPitch));
        return desiredPitch;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.restHood();
        //shooter.setSpeed(ShooterConstants.restingAngularVelocity);
        shooter.stopFlywheel();
        shooter.setFeederDutyCycle(FeederState.STOP);
        hopper.setDutyCycle(HopperState.STOP);
        CommandScheduler.getInstance().schedule(intake.new ChangeStates(IntakeState.BOUNCE_UP));
    }

    private boolean isAtDesiredRotation(Angle angle) {
        return drivetrain.getState().Pose.getRotation().getMeasure().isNear(angle, Degrees.of(5));
    }

    /** @return currently returns theoretical max that declines at a rate of 0.1 m/s (to simulate shooter slowing down over time), but when implemented with shooter will return current launch velocity based on shooter angular velocity */
    private double getTangentialVelocity() {
        //currently returns theoretical max that declines at a rate of 0.1 m/s
        return Units.feetToMeters(29) - (MathSharedStore.getTimestamp() - initializeTime) * 0.1;
    }
}
