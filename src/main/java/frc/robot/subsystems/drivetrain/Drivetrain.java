package frc.robot.subsystems.drivetrain;

import static frc.robot.operator.OperatorConstants.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AllianceUtil;
import frc.robot.operator.OperatorConstants.DriveFlag;
import frc.robot.operator.OperatorModality;

public class Drivetrain extends CommandSwerveDrivetrain {

    public final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleDrive =
        new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(POINT_kP, POINT_kI, POINT_kD)
            .withTargetRateFeedforward(POINT_FEED_FORWARD);
    public final SwerveRequest.FieldCentricFacingAngle trajectoryFacingAngleDrive =
        new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(POINT_kP, POINT_kI, POINT_kD)
            .withTargetRateFeedforward(POINT_FEED_FORWARD);
    public final SwerveRequest.SwerveDriveBrake brakeDrive = new SwerveRequest.SwerveDriveBrake();
    public final SwerveRequest.PointWheelsAt pointDrive = new SwerveRequest.PointWheelsAt();
    public final SwerveRequest.Idle idleDrive = new SwerveRequest.Idle();

    private final OperatorModality operatorModality;

    private final DriveFlags driveFlags = new DriveFlags();

    public Drivetrain(OperatorModality operatorModality) {
        super(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        );
        this.operatorModality = operatorModality;
    }

    public Command createDriveCommand(DriveMode driveMode) {
        return new Drive(this, driveMode);
    }

    public DriveFlags getDriveFlags() {
        return driveFlags;
    }

    public double getMaxTranslationSpeed() {
        return MAX_TRANSLATION_SPEED * (driveFlags.getValue(DriveFlag.SLOW_MODE) ? SLOW_MODE_TRANSLATION_FACTOR : 1);
    }

    public double getMaxRotationSpeed() {
        return MAX_ROTATION_SPEED * (driveFlags.getValue(DriveFlag.SLOW_MODE) ? SLOW_MODE_ROTATION_FACTOR : 1);
    }

    public Translation2d getInputVelocityTranslation(boolean fieldRelative) {
        return getInputTranslation(fieldRelative).times(getMaxTranslationSpeed());
    }

    public double getInputVelocityX(boolean fieldRelative) {
        return getInputX(fieldRelative) * getMaxTranslationSpeed();
    }

    public double getInputVelocityY(boolean fieldRelative) {
        return getInputY(fieldRelative) * getMaxTranslationSpeed();
    }

    public double getInputVelocityRotation() {
        return getInputRotation() * getMaxRotationSpeed();
    }

    public Translation2d getInputTranslation(boolean fieldRelative) {
        Translation2d rawInput = getRawInputTranslation(fieldRelative);
        Vector<N2> filteredInputVector = rawInput.toVector();
        filteredInputVector = MathUtil.applyDeadband(filteredInputVector, PRIMARY_TRANSLATION_DEADBAND, 1);

        // apply max radius
        filteredInputVector = filteredInputVector.div(PRIMARY_TRANSLATION_RADIUS);

        // apply exponent
        if (filteredInputVector.norm() > 0.0) {
            filteredInputVector = filteredInputVector
                .unit()
                .times(Math.pow(filteredInputVector.norm(), PRIMARY_TRANSLATION_EXPONENT));
        }

        // clamp values
        if (filteredInputVector.norm() > 1.0) {
            filteredInputVector = filteredInputVector.div(filteredInputVector.norm());
        }

        return new Translation2d(filteredInputVector);
    }

    public double getInputX(boolean fieldRelative) {
        return getInputTranslation(fieldRelative).getX();
    }

    public double getInputY(boolean fieldRelative) {
        return getInputTranslation(fieldRelative).getY();
    }

    public double getInputRotation() {
        double rawInput = getRawInputRotation();
        double filteredInput = MathUtil.applyDeadband(Math.abs(rawInput), PRIMARY_ROTATION_DEADBAND, 1);
        return Math.abs(Math.pow(filteredInput, PRIMARY_ROTATION_EXPONENT)) * Math.signum(rawInput);
    }

    /**
     * @return {@Code true} if there is no joystick input and the desired rotation
     *         has been reached
     */
    public boolean isDriveIdle() {
        return getInputTranslation(false).getNorm() == 0.0 && getInputRotation() == 0.0;
    }

    public boolean canAutoBrake() {
        return driveFlags.getValue(DriveFlag.AUTO_BRAKE) && isDriveIdle();
    }

    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(() -> getState().Pose, this::resetPose, this::followPath, true, this, trajLogger);
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * if autoPathAutoAimMode is true and the setAutonomousAutoAimCommand() method
     * was used to set the autoAimCommand, it will use the omega from the auto aim
     * command to aim at the hub
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        // current robot pose
        var pose = getState().Pose;
        // choreo calculated target speeds (time based)
        var targetSpeeds = sample.getChassisSpeeds();
        // calculate translation velocities with target speeds and PID to correct for
        // error
        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(pose.getX(), sample.x);
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(pose.getY(), sample.y);
        targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
            pose.getRotation().getRadians(),
            sample.heading
        );
        // send the calculated speeds to the drivetrain
        setControl(
            m_pathApplyFieldSpeeds
                .withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    private Translation2d getRawInputTranslation(boolean fieldRelative) {
        Translation2d rawInput = new Translation2d(getRawInputDriveX(), getRawInputDriveY());
        if (fieldRelative && isInverted()) {
            rawInput = rawInput.times(-1);
        }
        return rawInput;
    }

    private double getRawInputDriveX() {
        return operatorModality.translateX();
    }

    private double getRawInputDriveY() {
        return operatorModality.translateY();
    }

    private double getRawInputRotation() {
        return operatorModality.rotation();
    }

    private boolean isInverted() {
        return AllianceUtil.getAlliance() == Alliance.Red;
    }
}
