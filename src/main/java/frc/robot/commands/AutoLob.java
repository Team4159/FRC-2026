package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.FuelSimulation;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.HopperConstants.HopperState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.ShooterConstants.AutoAimStatus;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDStatusSupplier;
import frc.robot.subsystems.Shooter;

public class AutoLob extends Command {
    //Subsystems
    private final Drivetrain drivetrain;
    private final LEDs leds;
    private final Shooter shooter;
    private final Hopper hopper;
    private final Intake intake;

    private Timer timer;
    
    /** target pose2d (the hub based on alliance) */
    private Pose2d target;
    
    /** Robot pose when adjusted for distance traveled at current velocity during fuel TOF. Used for shooting while moving calculations. */
    private Pose2d adjustedRobotPose;

    /** used to send the angle to the auto path controller for use in auto period */
    private double desiredOmega;

    /** if the robot is in autonomous mode it will not apply speeds or require the drivetrain so different drive logic (ex. Choreo) can be used. */
    private boolean autonomousMode;

    /** stores auto aim statuses (SHOOT, WAITING, OUTOFRANGE) and corresponding LEDStatus*/
    private AutoAimStatus autoAimStatus;
    /** LED status supplier used for changing the LED status */
    private LEDStatusSupplier ledStatusSupplier;

    /** The height difference between the robot and hub. Currently it is a constant (final) but may change later to allow for shooting while climbing.*/
    private final double height = 0;

    /** used to push adjusted robot pose to advantagescope robot sim */
    private StructPublisher<Pose2d> adjustedRobotPosePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("adjustedRobotPose", Pose2d.struct).publish();

    //advantagescope sim
    /** for sim testing to simulate loss of velocity */
    private double timeOffset;
    
    /** keeps track of when the last fuel was shot during sim */
    private double lastShoot = MathSharedStore.getTimestamp();

    /**@param drivetrain the CommandSwerveDrivetrain
     * @param autonomousMode if set to true the actual robot swerve control will be disabled and the robot desired omega will be returned by the getDesiredOmega() function
     * it will also no longer require the drivetrain because a different command will be running for the auto path control to work
     * otherwise this constructor without the doublesuppliers will set the robot translation velocities to 0, it is designed to be used for auto
     */
    public AutoLob(Drivetrain drivetrain, Shooter shooter, Hopper hopper, Intake intake, LEDs leds, boolean autonomousMode){
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.hopper = hopper;
        this.leds = leds;
        this.intake = intake;

        this.timer = new Timer();

        autoAimStatus = AutoAimStatus.WAITING;
        ledStatusSupplier = () -> {return autoAimStatus.ledStatus;};
        this.autonomousMode = autonomousMode;
        if(!autonomousMode) addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        //set adjusted robot pose to current robot pose initially
        //(need a baseline to get time from lookup table)
        //this.target = Constants.FieldConstants.hubLocations.get(DriverStation.getAlliance().orElse(Alliance.Blue));
        adjustedRobotPose = drivetrain.getState().Pose;

        CommandScheduler.getInstance().schedule(leds.new ChangeLEDStatusSupplier(ledStatusSupplier));
        CommandScheduler.getInstance().schedule(intake.new BounceIntake());

        timeOffset = MathSharedStore.getTimestamp();

        shooter.setSpeed(ShooterConstants.lobAngularVelocity);

        timer.reset();
    }

    @Override
    public void execute() {
        //recalculate lob position
        this.target = drivetrain.getState().Pose.nearest(Constants.FieldConstants.lobLocations.get(DriverStation.getAlliance().orElse(Alliance.Blue)));

        //calculate desired pitch for hood angle
        double desiredHoodAngle = getDesiredHoodPitch();

        double robotRelativeBallVelocityHorizontal = getSimLaunchVelocity() * Math.cos(desiredHoodAngle);
        double robotRelativeBallVelocityVertical = getSimLaunchVelocity() * Math.sin(desiredHoodAngle);

        for(int i = 0; i < 2; i++){
            //calculate TOF(used for calculating adjusted robot pose)
            double timeOfFlight = getTimeOfFlight(desiredHoodAngle, shooter.getFuelSpeed());
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

        //calculate robot theta based on adjusted robot pose
        //this allows for shooting while moving

        double desiredRobotAngle = target.getTranslation().minus(adjustedRobotPose.getTranslation()).getAngle()
                .getRadians();


        if (!timer.hasElapsed(ShooterConstants.backwardsTime)){
            //run neck backwards if at the beginning
            autoAimStatus = AutoAimStatus.WAITING;
            shooter.setFeederSpeed(FeederState.UNSTUCKFEEDER.percentage);
            hopper.setHopperSpeed(HopperState.STOP.percentage);
        }
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

        //rotate the swerve to the desired angle
        rotateSwerve(desiredRobotAngle);

        //set the desired hood angle
        shooter.adjustTrajectoryAngle(Radians.of(desiredHoodAngle));

        SmartDashboard.putBoolean("isAtPitch", shooter.isAtPitch());
        SmartDashboard.putBoolean("isatspeed", shooter.isAtSpeed());
        SmartDashboard.putBoolean("swerve isatangle", isAtDesiredRotation(Radians.of(desiredRobotAngle)));
        if(shooter.isAtPitch() && shooter.isAtSpeed() && isAtDesiredRotation(Radians.of(desiredRobotAngle))){
            autoAimStatus = AutoAimStatus.SHOOT;
        }
        else{
            autoAimStatus = AutoAimStatus.WAITING;
        }

        if(autoAimStatus == AutoAimStatus.SHOOT){
            shooter.setFeederSpeed(FeederState.FEED.percentage);
            hopper.setHopperSpeed(HopperState.FEED.percentage);
        }

        //AdvantageScope fuel simulation
        //get the current robot yaw angle
        double robotYaw = drivetrain.getState().Pose.getRotation().getRadians();

        //calculate field relative initial fuel velocities
        double vx = robotRelativeBallVelocityHorizontal*Math.cos(desiredRobotAngle)
                    + drivetrain.getState().Speeds.vxMetersPerSecond * Math.cos(robotYaw) - drivetrain.getState().Speeds.vyMetersPerSecond * Math.sin(robotYaw);

        double vy = robotRelativeBallVelocityHorizontal*Math.sin(desiredRobotAngle)
                    + drivetrain.getState().Speeds.vxMetersPerSecond * Math.sin(robotYaw) + drivetrain.getState().Speeds.vyMetersPerSecond * Math.cos(robotYaw);

        double vz = robotRelativeBallVelocityVertical;

        SmartDashboard.putString("Auto Aim Status", autoAimStatus.name());

        //only feed (shown by shooting fuel in simulation) if the status is "SHOOT"
        sim_shootFuel(vx, vy, vz);
    }

    /** @param desiredAngle the desired field relative angle for the drivetrain
     * This also translates the robot using the getInputX() and getInputY() functions in the Drivetrain class
     */
    private void rotateSwerve(double desiredAngle){
        //PID controller to calculate omega
        double omega = Constants.DrivetrainConstants.AutoAimRotationController.calculate(
                drivetrain.getState().Pose.getRotation().getRadians(), desiredAngle, Timer.getFPGATimestamp());
        //set ChassisSpeeds
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            drivetrain.getInputSpeedX(true) * DrivetrainConstants.kAutoAimInputMultiplier,
            drivetrain.getInputSpeedY(true) * DrivetrainConstants.kAutoAimInputMultiplier,
            omega);

        //only actually control the swerve if not in autonomousMode
        if (!autonomousMode) {
                drivetrain.setControl(drivetrain.fieldCentricDrive
                        .withVelocityX(chassisSpeeds.vxMetersPerSecond)
                        .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                        .withRotationalRate(omega));
                }
        //this is so that the desired omega can be used in the command that controls swerve in auto period
        desiredOmega = omega;
    }

    /** @param hoodPitch the current pitch of the hood
     * @param launchVelocity the current launch velocity (magnitude of linear velocity for the fuel)
     * @return the time of flight (TOF) of the fuel when shot at hoodPitch with launchVelocity. takes the height difference of the shooter and hub into account.
     */
    private double getTimeOfFlight(double hoodPitch, double launchVelocity){
        //initial y component of launch velocity
        double vy = launchVelocity * Math.sin(hoodPitch);
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
 
    /** @return the desired pitch for the hood based on the adjusted robot position */
    private double getDesiredHoodPitch(){
        // distance from robot to target
        Translation2d robotTranslation = adjustedRobotPose.getTranslation();
        double distance = robotTranslation.getDistance(target.getTranslation());
        double launchVelocity = shooter.getFuelSpeed();
        if(RobotBase.isSimulation()){
            launchVelocity = getSimLaunchVelocity();
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
        // if(desiredPitch > Constants.ShooterConstants.maxPitch){
        //     desiredPitch = Constants.ShooterConstants.maxPitch;
        // }
        SmartDashboard.putNumber("autoaim desired pitch", Units.radiansToDegrees(desiredPitch));
        return desiredPitch;
    }


    //used for auto
    public double getDesiredOmega(){
        return desiredOmega;
    }

    private boolean isAtDesiredRotation(Angle angle){
        return drivetrain.getState().Pose.getRotation().getMeasure().isNear(angle, Degrees.of(5));
    }

    //sim
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

    /** @return currently returns theoretical max that declines at a rate of 0.1 m/s (to simulate shooter slowing down over time), but when implemented with shooter will return current launch velocity based on shooter angular velocity */
    private double getSimLaunchVelocity(){
        //currently returns theoretical max that declines at a rate of 0.1 m/s
        return Units.feetToMeters(29) - (MathSharedStore.getTimestamp() - timeOffset) * 0.1;
    }

    @Override
    public void end(boolean interrupted){
        shooter.adjustHood(ShooterConstants.kRestingAngle);
        //shooter.setSpeed(ShooterConstants.restingAngularVelocity);
        shooter.stopShooter();
        shooter.setFeederSpeed(FeederState.STOP.percentage);
        hopper.setHopperSpeed(HopperState.STOP.percentage);
        CommandScheduler.getInstance().schedule(intake.new ChangeStates(IntakeState.BOUNCE_UP));
    }
}