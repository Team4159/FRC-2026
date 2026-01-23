package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private double goalPitch;

    public AutoAim(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
        this.controller = controller;
        this.drivetrain = drivetrain;
        this.target = Constants.FieldConstants.hubLocations.get(DriverStation.getAlliance().orElse(Alliance.Blue));
        SmartDashboard.putData("shoot trajectory", trajectory);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        //TODO set initial goalpitch based on a lookuptable
        goalPitch = 45;
    }

    @Override
    public void execute() {
        // old
        double desiredAngle = target.getTranslation().minus(drivetrain.getState().Pose.getTranslation()).getAngle()
                .getRadians();
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
        // once angle is aligned start shooting with shooter angled accordingly
        double pitch = getPitch();
        SmartDashboard.putNumber("pitch", pitch);
        double vx = velocity * Math.cos(pitch);
        double vy = velocity * Math.sin(pitch);
        double lastAngle = 0;
        for (int i = 0; i < trajectoryLigaments.length; i++) {
            double tScale = 10 / 60.0;
            double t0 = tScale * i;
            double t1 = tScale * (i + 1);
            double x0 = vx * t0;
            double x1 = vx * t1;
            double y0 = vy * t0 - 0.5 * g * t0 * t0;
            double y1 = vy * t1 - 0.5 * g * t1 * t1;
            double angle = Math.toDegrees(Math.atan2(y1 - y0, x1 - x0));
            trajectoryLigaments[i].setLength(Math.hypot(x1 - x0, y1 - y0));
            trajectoryLigaments[i].setAngle(angle - lastAngle);
            lastAngle = angle;
        }

        //new
        //calculate initial z velocity of the ball based on shooter velocity and hood angle
        //TODO use actual shooter velocity in calculation for more reliablity
        double vozB = Constants.ShooterConstants.ratio * Constants.ShooterConstants.launchVelocity * Math.sin(goalPitch);

        //calculate the change in vertical required to score
        double deltaZ = Constants.FieldConstants.hubZ - Constants.ShooterConstants.shootHeight;

        //calculate the final velocity of the ball
        double vfzB = Math.sqrt(vozB * vozB + 2 * Constants.FieldConstants.g * deltaZ);

        //calculate the time of flight
        double timeOfFlight = (vfzB - vozB)/Constants.FieldConstants.g;

        //calculate required x velocity
        double voxB = (target.getX() - drivetrain.getState().Pose.getX())/timeOfFlight - drivetrain.getState().Speeds.vxMetersPerSecond;

        //calculate required y velocity
        double voyB = (target.getY() - drivetrain.getState().Pose.getY())/timeOfFlight - drivetrain.getState().Speeds.vyMetersPerSecond;
    }

    private double getPitch() {
        // distance from robot to target
        Translation2d robotTranslation = drivetrain.getState().Pose.getTranslation();
        double distance = robotTranslation.getDistance(target.getTranslation());
        double height = Units.inchesToMeters(72);
        SmartDashboard.putNumber("height", height);
        // TODO: implement shooter logic
        return Math.atan((Math.pow(velocity, 2)
                + Math.sqrt(Math.pow(velocity, 4) - Math.pow(g * distance, 2) - 2 * g * height * Math.pow(velocity, 2)))
                / (g * distance));
    }
}

// package frc.robot.commands;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;

// public class AutoAim extends Command{
//     private CommandSwerveDrivetrain drivetrain;
//     private SwerveRequest.ApplyFieldSpeeds fieldCentric = new SwerveRequest.ApplyFieldSpeeds();
//     private Pose2d target;
//     private double goalPitch;

//     public AutoAim(CommandSwerveDrivetrain drivetrain){
//         this.drivetrain = drivetrain;
//         this.target = Constants.FieldConstants.hubLocations.get(DriverStation.getAlliance().orElse(Alliance.Blue));
//         addRequirements(drivetrain);
//     }

//     @Override
//     public void initialize(){
//         //set goalPitch to lookuptable initial value
//     }

//     @Override
//     public void execute(){
//         //old
//         // double desiredAngle = target.getTranslation().minus(drivetrain.getState().Pose.getTranslation()).getAngle().getRadians();
//         // SmartDashboard.putNumber("desiredAngle", Units.radiansToDegrees(desiredAngle));
//         // SmartDashboard.putNumber("current angle", drivetrain.getState().Pose.getRotation().getDegrees());

//         // double omega = Constants.DrivetrainConstants.AutoAimRotationController.calculate(drivetrain.getState().Pose.getRotation().getRadians(), desiredAngle);
//         // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, omega);
//         // SmartDashboard.putNumber("omega", omega);
//         // drivetrain.setControl(fieldCentric.withSpeeds(chassisSpeeds));
//         //once angle is aligned start shooting with shooter angled accordingly

//         //calculate initial z velocity of the ball based on shooter velocity and hood angle
//         //TODO use actual shooter velocity in calculation for more reliablity
//         double vozB = Constants.ShooterConstants.ratio * Constants.ShooterConstants.launchVelocity * Math.sin(goalPitch);

//         //calculate the change in vertical required to score
//         double deltaZ = Constants.FieldConstants.hubZ - Constants.ShooterConstants.shootHeight;

//         //calculate the final velocity of the ball
//         double vfzB = Math.sqrt(vozB * vozB + 2 * Constants.FieldConstants.g * deltaZ);

//         //calculate the time of flight
//         double timeOfFlight = (vfzB - vozB)/Constants.FieldConstants.g;

//         //calculate required x velocity
//         double voxB = (target.getX() - drivetrain.getState().Pose.getX())/timeOfFlight - drivetrain.getState().Speeds.vxMetersPerSecond;

//         //calculate required y velocity
//         double voyB = (target.getY() - drivetrain.getState().Pose.getY())/timeOfFlight - drivetrain.getState().Speeds.vyMetersPerSecond;

//     }

//     private double getPitch(){
//         //distance from robot to target
//         double distance = drivetrain.getState().Pose.getTranslation().getDistance(target.getTranslation());
//         //TODO: implement shooter logic
//     }
// }