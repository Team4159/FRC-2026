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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.lib.GraphMechanism;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAim extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest.ApplyFieldSpeeds fieldCentric = new SwerveRequest.ApplyFieldSpeeds();
    private Pose2d target;
    private CommandXboxController controller;
    private GraphMechanism trajectoryVisual = new GraphMechanism("Shoot Trajectory");
    private double velocity = 10; // example value
    private double g = 9.81;

    public AutoAim(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
        this.controller = controller;
        this.drivetrain = drivetrain;
        this.target = Constants.FieldConstants.hubLocations.get(DriverStation.getAlliance().orElse(Alliance.Blue));
        addRequirements(drivetrain);
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
        trajectoryVisual.update(t -> new Translation2d(vx * t, vy * t - 0.5 * g * t * t));
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
