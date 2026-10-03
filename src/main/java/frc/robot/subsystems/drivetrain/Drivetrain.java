package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AllianceUtil;
import frc.robot.operator.OperatorConstants;
import frc.robot.operator.OperatorConstants.DriveFlag;
import frc.robot.operator.OperatorConstants.DriveMode;
import frc.robot.operator.OperatorModality;

public class Drivetrain extends CommandSwerveDrivetrain {

    public final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleDrive =
        new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(DrivetrainConstants.POINT_kP, DrivetrainConstants.POINT_kI, DrivetrainConstants.POINT_kD)
            .withTargetRateFeedforward(DrivetrainConstants.POINT_FEED_FORWARD);
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

    @Override
    public void periodic() {
        super.periodic();
        Pose2d pose = getState().Pose;
        SmartDashboard.putNumberArray("Drivetrain/Pose", new double[] {
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians(),
        });
    }

    public Command createDriveCommand(DriveMode driveMode) {
        return new Drive(this, driveMode);
    }

    public DriveFlags getDriveFlags() {
        return driveFlags;
    }

    public double getMaxTranslationSpeed() {
        return DrivetrainConstants.MAX_TRANSLATION_SPEED;
    }

    public double getMaxRotationSpeed() {
        return DrivetrainConstants.MAX_ROTATION_SPEED;
    }

    public double getTranslationSpeedFactor() {
        double factor = 1.0;
        if (driveFlags.getValue(DriveFlag.SLOW_MODE)) {
            factor *= OperatorConstants.SLOW_MODE_TRANSLATION_FACTOR;
        }
        if (driveFlags.getValue(DriveFlag.ALIGN_MODE)) {
            factor *= OperatorConstants.ALIGN_MODE_TRANSLATION_FACTOR;
        }
        return factor;
    }

    public double getRotationSpeedFactor() {
        double factor = 1.0;
        if (driveFlags.getValue(DriveFlag.SLOW_MODE)) {
            factor *= OperatorConstants.SLOW_MODE_ROTATION_FACTOR;
        }
        if (driveFlags.getValue(DriveFlag.ALIGN_MODE)) {
            factor *= OperatorConstants.ALIGN_MODE_ROTATION_FACTOR;
        }
        return factor;
    }

    public Translation2d getInputVelocityTranslation(boolean fieldRelative) {
        return getInputTranslation(fieldRelative).times(getMaxTranslationSpeed() * getTranslationSpeedFactor());
    }

    public double getInputVelocityX(boolean fieldRelative) {
        return getInputX(fieldRelative) * getMaxTranslationSpeed() * getTranslationSpeedFactor();
    }

    public double getInputVelocityY(boolean fieldRelative) {
        return getInputY(fieldRelative) * getMaxTranslationSpeed() * getTranslationSpeedFactor();
    }

    public double getInputVelocityRotation() {
        return getInputRotation() * getMaxRotationSpeed() * getRotationSpeedFactor();
    }

    public Translation2d getInputTranslation(boolean fieldRelative) {
        Translation2d rawInput = getRawInputTranslation(fieldRelative);
        return DriverInputProcessor.translation(
            rawInput.getX(),
            rawInput.getY(),
            OperatorConstants.PRIMARY_TRANSLATION_DEADBAND,
            OperatorConstants.PRIMARY_TRANSLATION_RADIUS,
            OperatorConstants.PRIMARY_TRANSLATION_EXPONENT
        );
    }

    public double getInputX(boolean fieldRelative) {
        return getInputTranslation(fieldRelative).getX();
    }

    public double getInputY(boolean fieldRelative) {
        return getInputTranslation(fieldRelative).getY();
    }

    public double getInputRotation() {
        return DriverInputProcessor.rotation(
            getRawInputRotation(),
            OperatorConstants.PRIMARY_ROTATION_DEADBAND,
            OperatorConstants.PRIMARY_ROTATION_EXPONENT
        );
    }

    /**
     * @return {@Code true} if there is no joystick input and the desired rotation
     *         has been reached
     */
    public boolean isInputIdle() {
        return getInputTranslation(false).getNorm() == 0.0 && getInputRotation() == 0.0;
    }

    public boolean canAutoBrake() {
        return driveFlags.getValue(DriveFlag.AUTO_BRAKE) && isInputIdle();
    }

    private Translation2d getRawInputTranslation(boolean fieldRelative) {
        Translation2d rawInput = new Translation2d(getRawInputX(), getRawInputY());
        if (fieldRelative && isInverted()) {
            rawInput = rawInput.times(-1);
        }
        return rawInput;
    }

    private double getRawInputX() {
        return operatorModality.translateX();
    }

    private double getRawInputY() {
        return operatorModality.translateY();
    }

    private double getRawInputRotation() {
        return operatorModality.rotation();
    }

    private boolean isInverted() {
        return AllianceUtil.getAlliance() == Alliance.Red;
    }
}
