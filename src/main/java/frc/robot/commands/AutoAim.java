package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.Constants;
import frc.robot.Constants.JoeLookupTableConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FeederConstants.FeederState;
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
    //Subsystems
    private final Drivetrain drivetrain;
    private final LEDs leds;
    private final Shooter shooter;
    private final Hopper hopper;
    private final Intake intake;

    /** Field relative swerve request used to drive the drivetrain with primary controller if not in autonomous mode.*/
    private SwerveRequest.ApplyFieldSpeeds fieldCentric = new SwerveRequest.ApplyFieldSpeeds();
    
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

    //advantagescope sim
    /** for sim testing to simulate loss of velocity */
    private double timeOffset;
    
    /** keeps track of when the last fuel was shot during sim */
    private double lastShoot = MathSharedStore.getTimestamp();

    /** used to push adjusted robot pose to advantagescope robot sim */
    private StructPublisher<Pose2d> adjustedRobotPosePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("adjustedRobotPose", Pose2d.struct).publish();

    /**@param drivetrain the CommandSwerveDrivetrain
     * @param autonomousMode if set to true the actual robot swerve control will be disabled and the robot desired omega will be returned by the getDesiredOmega() function
     * it will also no longer require the drivetrain because a different command will be running for the auto path control to work
     * otherwise this constructor without the doublesuppliers will set the robot translation velocities to 0, it is designed to be used for auto
     */
    public AutoAim(Drivetrain drivetrain, Shooter shooter, Hopper hopper, Intake intake, LEDs leds, boolean autonomousMode){
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.hopper = hopper;
        this.intake = intake;
        this.leds = leds;

        autoAimStatus = AutoAimStatus.WAITING;
        ledStatusSupplier = () -> {return autoAimStatus.ledStatus;};
        this.autonomousMode = autonomousMode;
        if(!autonomousMode) addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        //set adjusted robot pose to current robot pose initially
        //(need a baseline to get time from lookup table)
        this.target = Constants.FieldConstants.hubLocations.get(DriverStation.getAlliance().orElse(Alliance.Blue));
        adjustedRobotPose = drivetrain.getState().Pose;
        //used to simulate loss of shooter velocity over time for sim
        timeOffset = MathSharedStore.getTimestamp();

        CommandScheduler.getInstance().schedule(leds.new ChangeLEDStatusSupplier(ledStatusSupplier));
        CommandScheduler.getInstance().schedule(intake.new BounceIntake());

        shooter.setSpeed(ShooterConstants.shooterAngularVelocity);
    }

    @Override
    public void execute() {
        //calculate desired pitch for hood angle
        double desiredHoodAngle = getAdjustedHoodPitch(JoeLookupTable.getShotData(Meters.of(getDistanceFromHub())).getAngleRadians(), getDistanceFromHub());
        double timeOfFlight = JoeLookupTable.getShotData(Meters.of(getDistanceFromHub())).getTimeSeconds();

        double robotRelativeBallVelocityHorizontal = getLaunchVelocity() * Math.cos(desiredHoodAngle);
        double robotRelativeBallVelocityVertical = getLaunchVelocity() * Math.sin(desiredHoodAngle);

        //Transform of the current robot pose to the adjusted robot pose
        Transform2d adjustedRobotPoseTransform = new Transform2d(
                drivetrain.getState().Speeds.vxMetersPerSecond * timeOfFlight,
                drivetrain.getState().Speeds.vyMetersPerSecond * timeOfFlight,
                new Rotation2d());

        //add the distance traveled during TOF to current robot pose to get the adjusted robot pose
        //this will be used for shooting while moving adjustment
        adjustedRobotPose = drivetrain.getState().Pose.plus(adjustedRobotPoseTransform);

        //check if in range, return if out of range
        if(getDistanceFromHub() > JoeLookupTableConstants.kMaxDistance.in(Meters)){
            autoAimStatus = AutoAimStatus.OUTOFRANGE;
            return;
        }

        //send adjusted robot pose to advantageScope(for sim testing)
        adjustedRobotPosePublisher.set(adjustedRobotPose);

        //calculate robot theta based on adjusted robot pose
        //this allows for shooting while moving
        double desiredRobotAngle = target.getTranslation().minus(adjustedRobotPose.getTranslation()).getAngle()
                .getRadians();

        //rotate the swerve to the desired angle
        rotateSwerve(desiredRobotAngle);

        //set the desired hood angle
        shooter.adjustTrajectoryAngle(Radians.of(desiredHoodAngle));

        //for sim for now will implement actual tolerances later
        SmartDashboard.putBoolean("isAtPitch", shooter.isAtPitch());
        SmartDashboard.putBoolean("isatspeed", shooter.isAtSpeed());
        SmartDashboard.putBoolean("swerve isatangle", drivetrain.isAtDesiredRotation());
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
        if(autoAimStatus.name() == AutoAimStatus.SHOOT.name())
            sim_shootFuel(vx, vy, vz);

        SmartDashboard.putNumber("distance from hub", getDistanceFromHub());
        SmartDashboard.putNumber("autoaim desired pitch", Units.radiansToDegrees(desiredHoodAngle));
    }

    /** @param desiredAngle the desired field relative angle for the drivetrain
     * This also translates the robot using the getInputX() and getInputY() functions in the Drivetrain class
     */
    private void rotateSwerve(double desiredAngle){
        //PID controller to calculate omega
        double omega = Constants.DrivetrainConstants.AutoAimRotationController.calculate(
                drivetrain.getState().Pose.getRotation().getRadians(), desiredAngle, Timer.getFPGATimestamp());
        //set ChassisSpeeds
        System.out.println(omega);
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                drivetrain.getInputX(true)
                        * drivetrain.getMaxTranslationSpeed(),
                drivetrain.getInputY(true)
                        * drivetrain.getMaxTranslationSpeed(),
                omega);

        //only actually control the swerve if not in autonomousMode
        if(!autonomousMode) drivetrain.setControl(fieldCentric.withSpeeds(chassisSpeeds));
        //this is so that the desired omega can be used in the command that controls swerve in auto period
        desiredOmega = omega;
    }

    private boolean isAtDesiredRotation(Angle angle){
        return drivetrain.getState().Pose.getRotation().getMeasure().isNear(angle, Degrees.of(5));
    }
 
    /** @return the adjusted hood pitch based on wheel velocity
     * @param distance distance from hub
    */
    private double getAdjustedHoodPitch(double maxSpeedPitch, double distance){
        double error = Units.feetToMeters(29) - getLaunchVelocity();
        return maxSpeedPitch - error * (JoeLookupTableConstants.kShooterVelocityCorrection + distance * JoeLookupTableConstants.kShooterDistanceVelocityCorrection);
    }

    /** Units: meters */
    private double getDistanceFromHub(){
        return drivetrain.getState().Pose.getTranslation().getDistance(target.getTranslation());
    }

    /** @return currently returns theoretical max that declines at a rate of 0.1 m/s (to simulate shooter slowing down over time), but when implemented with shooter will return current launch velocity based on shooter angular velocity */
    private double getLaunchVelocity(){
        //currently returns theoretical max that declines at a rate of 0.1 m/s
        return Units.feetToMeters(29) - (MathSharedStore.getTimestamp() - timeOffset) * 0.1;
    }

    /** 
     * AdvantageScope fuel shooting simulation
     * @param vx initial field relative fuel velocity x component
     * @param vy initial field relative fuel velocity y component
     * @param vz initial field relative fuel velocity z component (FuelSimulation class will simulate gravity)
     */
    private void sim_shootFuel(double vx, double vy, double vz) {
        if (!RobotBase.isSimulation() || MathSharedStore.getTimestamp() - lastShoot <= 1.0 / 10.0) {
            return;
        }
        // FuelSimulation.getInstance().shootFuel(
        //         new Translation3d(drivetrain.getState().Pose.getTranslation().getX(),
        //                 drivetrain.getState().Pose.getTranslation().getY(), 0),
        //         new Translation3d(vx,
        //                 vy, vz),
        //         new Translation3d(0, 0, 0));
        lastShoot = MathSharedStore.getTimestamp();
    }

    //used for auto
    public double getDesiredOmega(){
        return desiredOmega;
    }

     @Override
    public void end(boolean interrupted){
        shooter.adjustHood(Degrees.of(2));
        shooter.stopShooter();
        shooter.setFeederSpeed(FeederState.STOP.percentage);
        hopper.setHopperSpeed(HopperState.STOP.percentage);
        CommandScheduler.getInstance().schedule(intake.new ChangeStates(IntakeState.DOWN_OFF));
    }
}