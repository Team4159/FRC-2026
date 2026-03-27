package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.FuelSimulation;
import frc.lib.HIDRumble;
import frc.lib.HIDRumble.RumbleRequest;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.JoeLookupTableConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HopperConstants.HopperState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.ShooterConstants.AutoAimStatus;
import frc.robot.lib.JoeLookupTable;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDStatusSupplier;
import frc.robot.subsystems.Shooter;

public class AutoAim extends Command {
    // Subsystems
    private final Drivetrain drivetrain;
    private final LEDs leds;
    private final Shooter shooter;
    private final Hopper hopper;
    private final Intake intake;

    private Timer timer;

    /**
     * Field relative swerve request used to drive the drivetrain with primary
     * controller if not in autonomous mode.
     */

    /** target pose2d (the hub based on alliance) */
    private Pose2d target;

    /**
     * Robot pose when adjusted for distance traveled at current velocity during
     * fuel TOF. Used for shooting while moving calculations.
     */
    private Pose2d adjustedRobotPose;

    /** used to send the angle to the auto path controller for use in auto period */
    private double desiredOmega;

    /**
     * if the robot is in autonomous mode it will not apply speeds or require the
     * drivetrain so different drive logic (ex. Choreo) can be used.
     */
    private boolean autonomousMode;

    /**
     * tells the robot to stop running swerve PID if already within tolerance to prevent jitter
     */
    private boolean aimFinished;

    /**
     * stores auto aim statuses (SHOOT, WAITING, OUTOFRANGE) and corresponding
     * LEDStatus
     */
    private AutoAimStatus autoAimStatus;
    /** LED status supplier used for changing the LED status */
    private LEDStatusSupplier ledStatusSupplier;

    // advantagescope sim
    /** keeps track of when the last fuel was shot during sim */
    private double lastShoot = MathSharedStore.getTimestamp();

    private final double height = FieldConstants.hubZ - Units.inchesToMeters(20);

    /** used to push adjusted robot pose to advantagescope robot sim */
    private StructPublisher<Pose2d> adjustedRobotPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("adjustedRobotPose", Pose2d.struct).publish();

    private Optional<CommandXboxController> feedbackController;

    /**
     * @param drivetrain     the CommandSwerveDrivetrain
     * @param autonomousMode if set to true the actual robot swerve control will be
     *                       disabled and the robot desired omega will be returned
     *                       by the getDesiredOmega() function
     *                       it will also no longer require the drivetrain because a
     *                       different command will be running for the auto path
     *                       control to work
     *                       otherwise this constructor without the doublesuppliers
     *                       will set the robot translation velocities to 0, it is
     *                       designed to be used for auto
     */
    public AutoAim(Drivetrain drivetrain, Shooter shooter, Hopper hopper, Intake intake, LEDs leds,
            boolean autonomousMode, Optional<CommandXboxController> feedbackController) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.hopper = hopper;
        this.intake = intake;
        this.leds = leds;
        this.feedbackController = feedbackController;

        autoAimStatus = AutoAimStatus.WAITING;
        ledStatusSupplier = () -> {
            return autoAimStatus.ledStatus;
        };
        this.autonomousMode = autonomousMode;
        if (!autonomousMode)
            addRequirements(drivetrain, shooter, hopper);

        timer = new Timer();
    }

    @Override
    public void initialize() {
        // set adjusted robot pose to current robot pose initially
        // (need a baseline to get time from lookup table)
        this.target = Constants.FieldConstants.hubLocations.get(DriverStation.getAlliance().orElse(Alliance.Blue));
        adjustedRobotPose = drivetrain.getState().Pose;
        // used to simulate loss of shooter velocity over time for sim
        //timeOffset = MathSharedStore.getTimestamp();

        CommandScheduler.getInstance().schedule(leds.new ChangeLEDStatusSupplier(ledStatusSupplier));
        CommandScheduler.getInstance().schedule(intake.new BounceIntake());

        aimFinished = false;

        timer.reset();
    }

    @Override
    public void execute() {
         //calculate desired pitch for hood angle
        Angle desiredHoodAngle = getDesiredHoodPitch();
        AngularVelocity desiredShooterAngularVelocity;

        for(int i = 0; i < 2; i++){
            desiredShooterAngularVelocity = JoeLookupTable.getDesiredAngularVelocity(Meters.of(getDistanceFromHub()));
            //calculate TOF(used for calculating adjusted robot pose)
            double timeOfFlight = getTimeOfFlight(desiredHoodAngle, shooter.getFuelSpeed());
            if(RobotBase.isSimulation()){
                timeOfFlight = getTimeOfFlight(desiredHoodAngle, getLaunchVelocity(desiredShooterAngularVelocity));
            }
            //calculate the distance traveled by the robot during the time of flight
            Transform2d adjustedRobotPoseTransform = new Transform2d(
                drivetrain.getState().Speeds.vxMetersPerSecond * timeOfFlight,
                drivetrain.getState().Speeds.vyMetersPerSecond * timeOfFlight,
                new Rotation2d());
            //add the distance traveled during TOF to current robot pose to get the adjusted robot pose
            //this will be used for shooting while moving adjustment
            adjustedRobotPose = drivetrain.getState().Pose.plus(adjustedRobotPoseTransform);

            //recalculate desired hood angle with new adjustedPose (converges)
            desiredHoodAngle = getDesiredHoodPitch();

            //send adjusted robot pose to advantageScope(for sim testing)
            adjustedRobotPosePublisher.set(adjustedRobotPose);
        }

        //get desired angular velocity from lookup table
        desiredShooterAngularVelocity = JoeLookupTable.getDesiredAngularVelocity(Meters.of(getDistanceFromHub()));

        // check if in range, return if out of range
        if (getDistanceFromHub() > JoeLookupTableConstants.kMaxDistance.in(Meters)) {
            autoAimStatus = AutoAimStatus.OUTOFRANGE;
            CommandScheduler.getInstance().cancel(this);
            return;
        }

        // calculate robot theta based on adjusted robot pose
        // this allows for shooting while moving
        double desiredRobotAngle = target.getTranslation().minus(adjustedRobotPose.getTranslation()).getAngle().getRadians();

        // rotate the swerve to the desired angle
        rotateSwerve(desiredRobotAngle);

        // set the desired hood angle
        shooter.adjustTrajectoryAngle(desiredHoodAngle);
        shooter.setSpeed(desiredShooterAngularVelocity);

        //send tolerances to smart dashboard
        SmartDashboard.putBoolean("isAtPitch", shooter.isAtPitch());
        SmartDashboard.putBoolean("isatspeed", shooter.isAtSpeed());
        SmartDashboard.putBoolean("swerve isatangle", isAtDesiredRotation(Radians.of(desiredRobotAngle)));

        // if (!timer.hasElapsed(ShooterConstants.backwardsTime)){
        //     //run neck backwards if at the beginning
        //     autoAimStatus = AutoAimStatus.WAITING;
        //     shooter.setFeederSpeed(FeederState.UNSTUCKFEEDER.percentage);
        //     hopper.setHopperSpeed(HopperState.STOP.percentage);
        // }
        if (shooter.isAtPitch() && shooter.isAtSpeed() && isAtDesiredRotation(Radians.of(desiredRobotAngle))) {
            //shoot the fuel if at the right pitch
            autoAimStatus = AutoAimStatus.SHOOT;
            shooter.setFeederSpeed(FeederState.FEED.percentage);
            hopper.setHopperSpeed(HopperState.FEED.percentage);
        } else {
            //otherwise just wait
            autoAimStatus = AutoAimStatus.WAITING;
            shooter.setFeederSpeed(FeederState.STOP.percentage);
            hopper.setHopperSpeed(HopperState.STOP.percentage);
        }

        if(RobotBase.isSimulation()){
            double robotRelativeBallVelocityHorizontal = getLaunchVelocity(desiredShooterAngularVelocity) * Math.cos(desiredHoodAngle.in(Radians));
            double robotRelativeBallVelocityVertical = getLaunchVelocity(desiredShooterAngularVelocity) * Math.sin(desiredHoodAngle.in(Radians));

            // AdvantageScope fuel simulation
            // get the current robot yaw angle
            double robotYaw = drivetrain.getState().Pose.getRotation().getRadians();

            // calculate field relative initial fuel velocities
            double vx = robotRelativeBallVelocityHorizontal * Math.cos(desiredRobotAngle)
                    + drivetrain.getState().Speeds.vxMetersPerSecond * Math.cos(robotYaw)
                    - drivetrain.getState().Speeds.vyMetersPerSecond * Math.sin(robotYaw);

            double vy = robotRelativeBallVelocityHorizontal * Math.sin(desiredRobotAngle)
                    + drivetrain.getState().Speeds.vxMetersPerSecond * Math.sin(robotYaw)
                    + drivetrain.getState().Speeds.vyMetersPerSecond * Math.cos(robotYaw);

            double vz = robotRelativeBallVelocityVertical;

            sim_shootFuel(vx, vy, vz);
        }

        SmartDashboard.putString("Auto Aim Status", autoAimStatus.name());
        SmartDashboard.putNumber("distance from hub", getDistanceFromHub());
        SmartDashboard.putNumber("autoaim desired pitch", desiredHoodAngle.in(Degrees));
    }

    /**
     * @param desiredAngle the desired field relative angle for the drivetrain
     *                     This also translates the robot using the getInputX() and
     *                     getInputY() functions in the Drivetrain class
     */
    private void rotateSwerve(double desiredAngle) {
        // if (drivetrain.getInputTranslation(true).getNorm() == 0.0) {
        //     boolean aimingAtHub = isAtDesiredRotation(Radians.of(desiredAngle));
        //     boolean robotIsStill = Math.toDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond) <= 1;
        //     if (aimingAtHub && robotIsStill) {
        //         aimFinished = true;
        //     }
        // } else {
        //     aimFinished = false;
        // }

        double omega = Constants.DrivetrainConstants.AutoAimRotationController.calculate(
                drivetrain.getState().Pose.getRotation().getRadians(), desiredAngle, Timer.getFPGATimestamp());

        if (!aimFinished) {
            // PID controller to calculate omega
            // set ChassisSpeeds
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                    drivetrain.getInputX(true) * DrivetrainConstants.kAutoAimInputMultiplier,
                    drivetrain.getInputY(true) * DrivetrainConstants.kAutoAimInputMultiplier,
                    omega);

            if (!autonomousMode) {
                drivetrain.setControl(drivetrain.fieldCentricDrive
                        .withVelocityX(chassisSpeeds.vxMetersPerSecond)
                        .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                        .withRotationalRate(omega));
            }
            // this is so that the desired omega can be used in the command that controls
            // swerve in auto period
            desiredOmega = omega;
        } else {
            if (!autonomousMode) {
                drivetrain.setControl(drivetrain.brakeDrive);
                desiredOmega = 0.0;
            } else {
                desiredOmega = omega;
            }
        }
    }

    private double getLaunchVelocity(AngularVelocity desiredMotorVelocity) {
        double shooterOmega = desiredMotorVelocity.in(RadiansPerSecond) * ShooterConstants.ratio;

        double wheelTangentialSpeed = shooterOmega * ShooterConstants.kShooterWheelRadius.in(Meters);
        double rollerTangentialSpeed = shooterOmega * ShooterConstants.kShooterRollerRadius.in(Meters);

        return ShooterConstants.kShooterEfficiency * (wheelTangentialSpeed + rollerTangentialSpeed)/2;
    }

    /**
     * AdvantageScope fuel shooting simulation
     * 
     * @param vx initial field relative fuel velocity x component
     * @param vy initial field relative fuel velocity y component
     * @param vz initial field relative fuel velocity z component (FuelSimulation
     *           class will simulate gravity)
     */
    private void sim_shootFuel(double vx, double vy, double vz) {
        if (!RobotBase.isSimulation() || MathSharedStore.getTimestamp() - lastShoot <= 1.0 / 10.0) {
            return;
        }
        FuelSimulation.getInstance().shootFuel(
        new Translation3d(drivetrain.getState().Pose.getTranslation().getX(),
        drivetrain.getState().Pose.getTranslation().getY(), 0),
        new Translation3d(vx,
        vy, vz),
        new Translation3d(0, 0, 0));
        lastShoot = MathSharedStore.getTimestamp();
    }

    /**
     * 
     * @param hoodPitch what angle the ball is shot at from the horizontal
     * @param launchVelocity the velocity the ball will be shot at in m/s
     * @return the time of flight to the hub in seconds
     */
    private double getTimeOfFlight(Angle hoodPitch, double launchVelocity){
        //initial y component of launch velocity
        double vy = launchVelocity * Math.sin(hoodPitch.in(Radians));
        //the calculation is based on delta y = vy * TOF - (1/2)g * TOF^2 (where g is a positive constant)
        //the delta y for TOF would be the height
        //the equation then becomes 0 = -(1/2)g * TOF^2 + vy * TOF - height -> 0 = (1/2)g * TOF^2 - vy * TOF + height
        //then use quadratic formula and always add the radical to get the 2nd time the fuel is at the target height (so that it is on the way down)
        double radical = Math.sqrt(Math.pow(vy, 2) - 2 * Constants.FieldConstants.g * height);
        if(Double.isNaN(radical)){
            return 0;
        }
        double numerator = vy + radical;
        double time = numerator/Constants.FieldConstants.g;
        SmartDashboard.putNumber("time of flight", time);
        return time;
    }

    private boolean isAtDesiredRotation(Angle angle) {
        return drivetrain.getState().Pose.getRotation().getMeasure().isNear(angle, Degrees.of(5));
    }

    /** Units: meters */
    private double getDistanceFromHub() {
        return drivetrain.getState().Pose.getTranslation().getDistance(target.getTranslation());
    }

    // used for auto to feed the desired omega of the swerve into the choreo path follower
    public double getDesiredOmega() {
        return desiredOmega;
    }

    /** @return the desired pitch for the hood based on the adjusted robot position */
    private Angle getDesiredHoodPitch(){
        // distance from robot to target
        Translation2d robotTranslation = adjustedRobotPose.getTranslation();
        double distance = robotTranslation.getDistance(target.getTranslation());
        double launchVelocity = shooter.getFuelSpeed();
        if(RobotBase.isSimulation()){
            launchVelocity = getLaunchVelocity(JoeLookupTable.getDesiredAngularVelocity(Meters.of(getDistanceFromHub())));
        }
        double desiredPitch = 
            Math.atan((Math.pow(launchVelocity, 2)
                + Math.sqrt(Math.pow(launchVelocity, 4)
                        - Math.pow(FieldConstants.g * distance, 2)
                        - 2 * FieldConstants.g * height
                                * Math.pow(launchVelocity, 2)))
                / (FieldConstants.g * distance));

        if(Double.isNaN(desiredPitch)){
            //equation can only return angles from 45-90 deg (in radians of course), anything lower than that will be NaN
            //the minimum possible hood angle on the physical shooter is 45, so no additional calculation is needed, just set to 45
            desiredPitch = Units.degreesToRadians(45);
            autoAimStatus = AutoAimStatus.OUTOFRANGE;
        }
        if(desiredPitch > Constants.ShooterConstants.maxPitch.in(Radians)){
            desiredPitch = Constants.ShooterConstants.maxPitch.in(Radians);
        }
        SmartDashboard.putNumber("autoaim desired pitch", Units.radiansToDegrees(desiredPitch));
        return Radians.of(desiredPitch);
    }

    @Override
    public void end(boolean interrupted) {
        if (feedbackController.isPresent()) {
            HIDRumble.rumble(feedbackController.get().getHID(), new RumbleRequest(RumbleType.kLeftRumble, 0.5, 0.25));
        }
        shooter.adjustHood(Degrees.of(2));
        shooter.stopShooter();
        //shooter.setFeederSpeed(FeederState.STOP.percentage);
        hopper.setHopperSpeed(HopperState.STOP.percentage);
        CommandScheduler.getInstance().schedule(intake.new ChangeStates(IntakeState.BOUNCE_UP));
    }
}