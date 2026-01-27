package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.FuelSimulation;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAim extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest.ApplyFieldSpeeds fieldCentric = new SwerveRequest.ApplyFieldSpeeds();
    private Pose2d target;
    
    private DoubleSupplier robotXVelocitySupplier, robotYVelocitySupplier;
    
    private Pose2d adjustedRobotPose;

    private double lastShoot = MathSharedStore.getTimestamp();

    private StructPublisher<Pose2d> adjustedRobotPosePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("adjustedRobotPose", Pose2d.struct).publish();

    /**@param drivetrain the CommandSwerveDrivetrain
     * @param robotXVelocitySupplier a DoubleSupplier for providing the desired field relative robot velocity x component
     * @param robotYVelocitySupplier a DoubleSupplier for providing the desired field relative robot velocity y component
     */
    public AutoAim(CommandSwerveDrivetrain drivetrain, DoubleSupplier robotXVelocitySupplier, DoubleSupplier robotYVelocitySupplier) {
        this.drivetrain = drivetrain;
        this.robotXVelocitySupplier = robotXVelocitySupplier;
        this.robotYVelocitySupplier = robotYVelocitySupplier;
        this.target = Constants.FieldConstants.hubLocations.get(DriverStation.getAlliance().orElse(Alliance.Blue));
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        //set adjusted robot pose to current robot pose initially
        //(need a baseline to get time from lookup table)
        adjustedRobotPose = drivetrain.getState().Pose;
    }

    @Override
    public void execute() {
        //get distance from adjusted robot pose
        double distance = getDistanceFromTarget(adjustedRobotPose);

        //get time of flight from distance
        double timeOfFlight = getTimeOfFlight(distance);

        //maximum height of the ball during the trajectory
        double maxHeight = Units.inchesToMeters(70);

        //calculate adjusted robot pose

        //calculate the distance traveled by the robot during the time of flight
        Transform2d adjustedRobotPoseTransform = new Transform2d(
            drivetrain.getState().Speeds.vxMetersPerSecond * timeOfFlight,
            drivetrain.getState().Speeds.vyMetersPerSecond * timeOfFlight,
            new Rotation2d());

        //add the distance traveled during TOF to current robot pose to get the adjusted robot pose
        //this will be used for shooting while moving adjustment
        adjustedRobotPose = drivetrain.getState().Pose.plus(adjustedRobotPoseTransform);

        //calculate desired horizontal and vertical initial ball velocities
        //vx = distance/time
        double robotRelativeBallVelocityHorizontal = distance/timeOfFlight;
        //vy = (hmax - hstart)/time + (1/2)g*time (derived from hmax = hstart + vy*time - (1/2)g*time^2)
        double robotRelativeBallVelocityVertical = (maxHeight - Constants.ShooterConstants.shootHeight) / timeOfFlight + (0.5) * Constants.FieldConstants.g * timeOfFlight;

        //calculate hood angle from ball velocities
        double hoodAngle = Math.atan2(robotRelativeBallVelocityVertical, robotRelativeBallVelocityHorizontal);

        //calculate desired ball velocity 
        double robotRelativeBallVelocity = Math.sqrt(Math.pow(robotRelativeBallVelocityHorizontal, 2) + Math.pow(robotRelativeBallVelocityVertical, 2));

        //calculate desired wheel velocity
        double wheelVelocity = robotRelativeBallVelocity/Constants.ShooterConstants.ratio;

        //TODO implement shooter (hoodAngle, wheelVelocity) -> shooter setpoints

        //calculate robot theta based on adjusted robot pose
        //this allows for shooting while moving
        double desiredRobotAngle = target.getTranslation().minus(adjustedRobotPose.getTranslation()).getAngle()
                .getRadians();

        //rotate the swerve to the desired angle
        rotateSwerve(desiredRobotAngle);

        //send adjusted robot pose to advantageScope(for sim testing)
        adjustedRobotPosePublisher.set(adjustedRobotPose);

        //AdvantageScope fuel simulation
        //get the current robot yaw angle
        double robotYaw = drivetrain.getState().Pose.getRotation().getRadians();

        //calculate field relative initial fuel velocities
        double vx = robotRelativeBallVelocityHorizontal*Math.cos(desiredRobotAngle)
                    + drivetrain.getState().Speeds.vxMetersPerSecond * Math.cos(robotYaw) - drivetrain.getState().Speeds.vyMetersPerSecond * Math.sin(robotYaw);

        double vy = robotRelativeBallVelocityHorizontal*Math.sin(desiredRobotAngle)
                    + drivetrain.getState().Speeds.vxMetersPerSecond * Math.sin(robotYaw) + drivetrain.getState().Speeds.vyMetersPerSecond * Math.cos(robotYaw);

        double vz = robotRelativeBallVelocityVertical;

        sim_shootFuel(vx, vy, vz);
    }

    /** @param desiredAngle the desired field relative angle for the drivetrain
     * This also translates the robot using the DoubleSuppliers as well
     */
    private void rotateSwerve(double desiredAngle){
        //PID controller to calculate omega
        double omega = Constants.DrivetrainConstants.AutoAimRotationController.calculate(
                drivetrain.getState().Pose.getRotation().getRadians(), desiredAngle, Timer.getFPGATimestamp());
        //set ChassisSpeeds to the current values of the DoubleSuppliers for translation and omega for rotation
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                robotXVelocitySupplier.getAsDouble(),
                robotYVelocitySupplier.getAsDouble(),
                omega);
        drivetrain.setControl(fieldCentric.withSpeeds(chassisSpeeds));
    }

    /** @param robotPose the current field relative robot pose
     * @return the distance from the current target
     */
    private double getDistanceFromTarget(Pose2d robotPose){
        return robotPose.getTranslation().getDistance(target.getTranslation());
    }

    /** @param distance estimated robot distance from target 
     * @return the estimated time of flight of a fuel launched, using a lookuptable and linear interpolation
    */
    private double getTimeOfFlight(double distance){
        double closestDistance = 0, secondClosestDistance = 0;
        var lookupTable = Constants.ShooterConstants.timeLookupTable;
        var keyset = lookupTable.keySet();
        for(var key : keyset){
            double currentDistance = key;
            if(Math.abs(currentDistance - distance) < Math.abs(closestDistance - distance)){
                secondClosestDistance = closestDistance;
                closestDistance = currentDistance;
            }
            else if(Math.abs(currentDistance - distance) < Math.abs(secondClosestDistance - distance)){
                secondClosestDistance = currentDistance;
            }
        }
        //linear interpolation
        double difference = Math.abs(secondClosestDistance - closestDistance);
        double percentage = Math.abs(closestDistance - distance) / difference;
        return MathUtil.interpolate(lookupTable.get(closestDistance), lookupTable.get(secondClosestDistance), percentage);
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
        FuelSimulation.getInstance().shootFuel(
                new Translation3d(drivetrain.getState().Pose.getTranslation().getX(),
                        drivetrain.getState().Pose.getTranslation().getY(), 0),
                new Translation3d(vx,
                        vy, vz),
                new Translation3d(0, 0, 0));
        lastShoot = MathSharedStore.getTimestamp();
    }
}