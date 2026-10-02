package frc.robot.commands.shoot;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FuelSimulation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator.ShotCalculatorResult;

public abstract class Shoot extends Command {

    private double lastSimShoot = -1.0;

    protected boolean isReadyToShoot(Drivetrain drivetrain, Shooter shooter, Angle yaw) {
        return (
            shooter.isAtHoodPitch() && shooter.isAtFlywheelVelocity() && isAtDesiredRotation(drivetrain.getState(), yaw)
        );
    }

    protected boolean isAtDesiredRotation(SwerveDriveState state, Angle angle) {
        return state.Pose.getRotation().getMeasure().isNear(angle, DrivetrainConstants.AUTO_SHOOT_TOLERANCE);
    }

    /**
     * @param yaw the desired field relative angle for the drivetrain
     *                     This also translates the robot using the getInputX() and
     *                     getInputY() functions in the Drivetrain class
     */
    protected void rotateSwerve(Drivetrain drivetrain, double yaw) {
        SwerveDriveState state = drivetrain.getState();

        boolean aimFinished;
        if (drivetrain.getInputTranslation(true).getNorm() == 0.0) {
            boolean aimingAtHub = state.Pose.getRotation()
                .getMeasure()
                .isNear(Radians.of(yaw), DrivetrainConstants.AUTO_SHOOT_TOLERANCE);
            boolean robotIsStill = Math.toDegrees(state.Speeds.omegaRadiansPerSecond) <= 1;
            aimFinished = aimingAtHub && robotIsStill;
        } else {
            aimFinished = false;
        }

        if (aimFinished) {
            drivetrain.setControl(drivetrain.brakeDrive);
            return;
        }

        double vx = drivetrain.getInputX(true) * DrivetrainConstants.AUTO_SHOOT_INPUT_MULTIPLIER;
        double vy = drivetrain.getInputY(true) * DrivetrainConstants.AUTO_SHOOT_INPUT_MULTIPLIER;
        double omega = DrivetrainConstants.AUTO_SHOOT_ROTATION_CONTROLLER.calculate(
            state.Pose.getRotation().getRadians(),
            yaw
        );

        drivetrain.setControl(
            drivetrain.fieldCentricDrive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega)
        );
    }

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

        lastSimShoot = MathSharedStore.getTimestamp();

        double robotRelativeBallVelocityHorizontal =
            result.tangentialVelocity().baseUnitMagnitude() * Math.cos(result.pitch().baseUnitMagnitude());
        double robotRelativeBallVelocityVertical =
            result.tangentialVelocity().baseUnitMagnitude() * Math.sin(result.pitch().baseUnitMagnitude());

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
    }
}
