package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAim extends Command{
    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest.ApplyFieldSpeeds fieldCentric = new SwerveRequest.ApplyFieldSpeeds();
    private Pose2d target;

    public AutoAim(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
        this.target = Constants.FieldConstants.hubLocations.get(DriverStation.getAlliance().orElse(Alliance.Blue));
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        //old
        double desiredAngle = target.getTranslation().minus(drivetrain.getState().Pose.getTranslation()).getAngle().getRadians();
        SmartDashboard.putNumber("desiredAngle", Units.radiansToDegrees(desiredAngle));
        SmartDashboard.putNumber("current angle", drivetrain.getState().Pose.getRotation().getDegrees());

        double omega = Constants.DrivetrainConstants.AutoAimRotationController.calculate(drivetrain.getState().Pose.getRotation().getRadians(), desiredAngle);
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, omega);
        SmartDashboard.putNumber("omega", omega);
        drivetrain.setControl(fieldCentric.withSpeeds(chassisSpeeds));
        //once angle is aligned start shooting with shooter angled accordingly
    }

    private double getPitch(){
        //distance from robot to target
        double distance = drivetrain.getState().Pose.getTranslation().getDistance(target.getTranslation());
        //TODO: implement shooter logic
    }
}
