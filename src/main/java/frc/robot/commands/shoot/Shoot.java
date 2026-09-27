package frc.robot.commands.shoot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FuelSimulation;
import frc.robot.subsystems.shooter.ShotCalculator.ShotCalculatorResult;

public abstract class Shoot extends Command {

    private double lastSimShoot = -1.0;

    /**
     * AdvantageScope fuel shooting simulation
     *
     * @param vx initial field relative fuel velocity x component
     * @param vy initial field relative fuel velocity y component
     * @param vz initial field relative fuel velocity z component (FuelSimulation
     *           class will simulate gravity)
     */
    protected void simShoot(ShotCalculatorResult result, SwerveDriveState state) {
        if (!RobotBase.isSimulation() || MathSharedStore.getTimestamp() - lastSimShoot <= 1.0 / 10.0) {
            return;
        }
        double robotRelativeBallVelocityHorizontal =
            result.tangentialVelocity().baseUnitMagnitude() * Math.cos(result.pitch().baseUnitMagnitude());
        double robotRelativeBallVelocityVertical =
            result.tangentialVelocity().baseUnitMagnitude() * Math.sin(result.pitch().baseUnitMagnitude());

        // AdvantageScope fuel simulation
        // calculate field relative initial fuel velocities
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
        double vx =
            robotRelativeBallVelocityHorizontal * Math.cos(result.yaw().baseUnitMagnitude()) +
            fieldSpeeds.vxMetersPerSecond;

        double vy =
            robotRelativeBallVelocityHorizontal * Math.sin(result.yaw().baseUnitMagnitude()) +
            fieldSpeeds.vyMetersPerSecond;

        double vz = robotRelativeBallVelocityVertical;
        FuelSimulation.getInstance().shootFuel(
            new Translation3d(state.Pose.getTranslation().getX(), state.Pose.getTranslation().getY(), 0.0),
            new Translation3d(vx, vy, vz),
            new Translation3d(0.0, 0.0, 0.0)
        );
        lastSimShoot = MathSharedStore.getTimestamp();
    }
}
