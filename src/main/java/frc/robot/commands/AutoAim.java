package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.GraphMechanism;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoAim extends Command {
    private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private Drivetrain drivetrain;
    private SwerveRequest.ApplyFieldSpeeds fieldCentric = new SwerveRequest.ApplyFieldSpeeds();
    private Pose2d target;
    private GraphMechanism trajectoryVisual = new GraphMechanism("Shoot Trajectory");

    public AutoAim(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.target = Constants.FieldConstants.hubLocations
                .get(DriverStation.getAlliance().orElse(Alliance.Blue));
        this.trajectoryVisual.hide();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        trajectoryVisual.show();
    }

    @Override
    public void execute() {
        // old
        double desiredAngle = target.getTranslation().minus(drivetrain.getState().Pose.getTranslation())
                .getAngle()
                .getRadians();
        SmartDashboard.putNumber("desiredAngle", Units.radiansToDegrees(desiredAngle));
        SmartDashboard.putNumber("current angle", drivetrain.getState().Pose.getRotation().getDegrees());

        double omega = Constants.DrivetrainConstants.AutoAimRotationController.calculate(
                drivetrain.getState().Pose.getRotation().getRadians(), desiredAngle,
                Timer.getFPGATimestamp());
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                drivetrain.getInputX()
                        * maxSpeed,
                drivetrain.getInputY()
                        * maxSpeed,
                omega);
        SmartDashboard.putNumber("omega", omega);
        drivetrain.setControl(fieldCentric.withSpeeds(chassisSpeeds));
        // once angle is aligned start shooting with shooter angled accordingly
        double pitch = getPitch();
        SmartDashboard.putNumber("pitch", pitch);
        double vx = ShooterConstants.launchVelocity * Math.cos(pitch);
        double vy = ShooterConstants.launchVelocity * Math.sin(pitch);
        trajectoryVisual.update(t -> new Translation2d(vx * t, vy * t - 0.5 * FieldConstants.g * t * t));
    }

    @Override
    public void end(boolean interrupted) {
        trajectoryVisual.hide();
    }

    private double getPitch() {
        // distance from robot to target
        Translation2d robotTranslation = drivetrain.getState().Pose.getTranslation();
        double distance = robotTranslation.getDistance(target.getTranslation());
        double height = Units.inchesToMeters(72);
        SmartDashboard.putNumber("height", height);
        // TODO: implement shooter logic
        return Math.atan((Math.pow(ShooterConstants.launchVelocity, 2)
                + Math.sqrt(Math.pow(ShooterConstants.launchVelocity, 4)
                        - Math.pow(FieldConstants.g * distance, 2)
                        - 2 * FieldConstants.g * height
                                * Math.pow(ShooterConstants.launchVelocity, 2)))
                / (FieldConstants.g * distance));
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