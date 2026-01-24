package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAim extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest.ApplyFieldSpeeds fieldCentric = new SwerveRequest.ApplyFieldSpeeds();
    private Pose2d target;
    private CommandXboxController controller;
    private Mechanism2d trajectory = new Mechanism2d(1, 1);
    private MechanismLigament2d[] trajectoryLigaments = new MechanismLigament2d[20];
    {
        var root = trajectory.getRoot("shooter", 0.5, 0);
        MechanismLigament2d last = null;
        for (int i = 0; i < trajectoryLigaments.length; i++) {
            var ligament = new MechanismLigament2d(String.valueOf(i), 0, 0);
            trajectoryLigaments[i] = ligament;
            ligament.setLineWeight(2);
            if (last == null) {
                root.append(ligament);
            } else {
                last.append(ligament);
            }
            last = ligament;
        }
    }
    private double velocity = 10; // example value
    private double g = 9.81;
    private Pose2d adjustedRobotPose;

    StructPublisher<Pose2d> adjustedRobotPosePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("adjustedRobotPose", Pose2d.struct).publish();

    public AutoAim(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
        this.controller = controller;
        this.drivetrain = drivetrain;
        this.target = Constants.FieldConstants.hubLocations.get(DriverStation.getAlliance().orElse(Alliance.Blue));
        SmartDashboard.putData("shoot trajectory", trajectory);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        //set adjusted robot pose to current robot pose initially
        //(need a baseline to get time from lookup table)
        adjustedRobotPose = drivetrain.getState().Pose;
        System.out.println("set adjustedRobotPose");
    }

    @Override
    public void execute() {
        //get distance from adjusted robot pose
        double distance = getDistanceFromHub(adjustedRobotPose);
        //double distance = 1;
        System.out.println("getDistanceFromHub");

        //get time of flight from distance
        double timeOfFlight = getTimeOfFlight(distance);
        System.out.println("getTOF");

        //maximum height of the ball during the trajectory
        double maxHeight = Units.inchesToMeters(70);
        System.out.println("maxHeight");

        //Field2d adjustedRobotPoseF2d = new Field2d();
        //adjustedRobotPoseF2d.setRobotPose(adjustedRobotPose);
        //SmartDashboard.putData("adjustedPose", adjustedRobotPoseF2d);
        adjustedRobotPosePublisher.set(adjustedRobotPose);

        Transform2d adjustedRobotPoseTranslation = new Transform2d(
            drivetrain.getState().Speeds.vxMetersPerSecond * timeOfFlight,
            drivetrain.getState().Speeds.vyMetersPerSecond * timeOfFlight,
            new Rotation2d());

        adjustedRobotPose = drivetrain.getState().Pose.plus(adjustedRobotPoseTranslation);
        //offset robot pose based off current robot velocities and time
        // adjustedRobotPose = new Pose2d(
        // drivetrain.getState().Speeds.vxMetersPerSecond * timeOfFlight,
        // drivetrain.getState().Speeds.vyMetersPerSecond * timeOfFlight,
        // drivetrain.getState().Pose.getRotation());
        //double height = Units.inchesToMeters(distance)

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

        //calculate robot theta based on adjusted velocity
        double desiredRobotAngle = target.getTranslation().minus(adjustedRobotPose.getTranslation()).getAngle()
                .getRadians();

        //tell the swerve to rotate to the desired angle
        rotateSwerve(desiredRobotAngle);

        double vx = robotRelativeBallVelocityHorizontal*Math.sin(desiredRobotAngle)
                    + drivetrain.getState().Speeds.vxMetersPerSecond;
        System.out.println("vxrobotrelative:" + robotRelativeBallVelocityHorizontal*Math.sin(desiredRobotAngle));

        double vy = robotRelativeBallVelocityHorizontal*Math.cos(desiredRobotAngle)
                    + drivetrain.getState().Speeds.vyMetersPerSecond;
        System.out.println("vyrobotrelative:" + robotRelativeBallVelocityHorizontal*Math.cos(desiredRobotAngle));

        //AdvantageScope trajectory sim
        //TODO actually do the calculation correctly, need to account for the signs
        double vHorizontal = Math.sqrt(vx * vx + vy * vy);

        double vz = robotRelativeBallVelocityVertical;
        double lastAngle = 0;

        for (int i = 0; i < trajectoryLigaments.length; i++) {
            double tScale = 10 / 60.0;
            double t0 = tScale * i;
            double t1 = tScale * (i + 1);
            double x0 = vHorizontal * t0;
            double x1 = vHorizontal * t1;
            double y0 = vz * t0 - 0.5 * g * t0 * t0;
            double y1 = vz * t1 - 0.5 * g * t1 * t1;
            double angle = Math.toDegrees(Math.atan2(y1 - y0, x1 - x0));
            trajectoryLigaments[i].setLength(Math.hypot(x1 - x0, y1 - y0));
            trajectoryLigaments[i].setAngle(angle - lastAngle);
            lastAngle = angle;
        }
    }

    private void rotateSwerve(double desiredAngle){
        SmartDashboard.putNumber("desiredAngle", Units.radiansToDegrees(desiredAngle));
        SmartDashboard.putNumber("current angle", drivetrain.getState().Pose.getRotation().getDegrees());

        double omega = Constants.DrivetrainConstants.AutoAimRotationController.calculate(
                drivetrain.getState().Pose.getRotation().getRadians(), desiredAngle, Timer.getFPGATimestamp());
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                MathUtil.applyDeadband(Math.abs(controller.getLeftY()), 0.1) * Math.signum(controller.getLeftY())
                        * 4.54,
                MathUtil.applyDeadband(Math.abs(controller.getLeftX()), 0.1) * Math.signum(controller.getLeftX())
                        * 4.54,
                omega);
        SmartDashboard.putNumber("omega", omega);
        drivetrain.setControl(fieldCentric.withSpeeds(chassisSpeeds));
    }

    private double getDistanceFromHub(Pose2d robotPose){
        return robotPose.getTranslation().getDistance(target.getTranslation());
    }

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
        double difference = Math.abs(secondClosestDistance - closestDistance);
        double percentage = Math.abs(closestDistance - distance) / difference;
        //return MathUtil.interpolate(lookupTable.get(closestDistance), lookupTable.get(secondClosestDistance), percentage);
        System.out.println("closestDistance: " + closestDistance);
        return lookupTable.get(closestDistance);
    }

    // private double getPitch() {
    //     // distance from robot to target
    //     Translation2d robotTranslation = drivetrain.getState().Pose.getTranslation();
    //     double distance = robotTranslation.getDistance(target.getTranslation());
    //     double height = Units.inchesToMeters(72);
    //     SmartDashboard.putNumber("height", height);
    //     // TODO: implement shooter logic
    //     return Math.atan((Math.pow(velocity, 2)
    //             + Math.sqrt(Math.pow(velocity, 4) - Math.pow(g * distance, 2) - 2 * g * height * Math.pow(velocity, 2)))
    //             / (g * distance));
    // }
}