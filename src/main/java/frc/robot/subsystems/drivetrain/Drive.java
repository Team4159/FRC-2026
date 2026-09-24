package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.operator.OperatorConstants.ALIGN_MODE_SPEED_ROTATION_FACTOR;
import static frc.robot.operator.OperatorConstants.ALIGN_MODE_SPEED_TRANSLATION_FACTOR;
import static frc.robot.operator.OperatorConstants.INTAKE_ROTATION_INPUT_DEADZONE;
import static frc.robot.operator.OperatorConstants.TRENCH_ASSIST_ALIGN_STRENGTH;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.PoseUtil;
import frc.robot.Constants.FieldConstants;
import frc.robot.operator.OperatorConstants;
import frc.robot.operator.OperatorConstants.DriveFlag;
import frc.robot.operator.OperatorConstants.DriveMode;
import frc.robot.subsystems.hopper.HopperConstants;
import java.util.Optional;
import java.util.function.Supplier;

public class Drive extends Command {

    private final Drivetrain drivetrain;
    private final Supplier<SwerveRequest> driveSupplier;

    public Drive(Drivetrain drivetrain, DriveMode driveMode) {
        this.drivetrain = drivetrain;
        this.driveSupplier = getDriveSupplier(driveMode);
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setControl(driveSupplier.get());
    }

    private SwerveRequest getTeleopDrive() {
        if (drivetrain.canAutoBrake()) {
            return drivetrain.brakeDrive;
        }

        double maxTranslationSpeed = drivetrain.getMaxTranslationSpeed();
        double maxRotationSpeed = drivetrain.getMaxRotationSpeed();
        if (drivetrain.getDriveFlagValue(DriveFlag.SLOW_MODE) && drivetrain.getDriveFlagValue(DriveFlag.MANUAL_ALIGN)) {
            maxTranslationSpeed *= ALIGN_MODE_SPEED_TRANSLATION_FACTOR;
            maxRotationSpeed *= ALIGN_MODE_SPEED_ROTATION_FACTOR;
        }

        Translation2d inputSpeedTranslation;
        double inputSpeedRotation = drivetrain.getInputRotation() * maxRotationSpeed;
        if (drivetrain.getDriveFlagValue(DriveFlag.MANUAL_ALIGN)) {
            Translation2d input = drivetrain.getInputTranslation(true);
            double x = 0;
            double y = 0;
            if (input.getNorm() >= 0.0) {
                if (Math.abs(input.getAngle().getCos()) >= input.getNorm() / Math.sqrt(2)) {
                    x = Math.signum(input.getX());
                } else {
                    y = Math.signum(input.getY());
                }
            }
            inputSpeedTranslation = new Translation2d(x * maxTranslationSpeed, y * maxTranslationSpeed);
        } else {
            inputSpeedTranslation = drivetrain.getInputSpeedTranslation(true);
        }

        Optional<Rotation2d> desiredRotation = Optional.empty();
        if (
            drivetrain.getDriveFlagValue(DriveFlag.INTAKE_ASSIST) &&
            drivetrain.getInputTranslation(true).getNorm() >= INTAKE_ROTATION_INPUT_DEADZONE
        ) {
            Angle angle;
            int angleSign = (int) Math.signum(drivetrain.getInputRotation());
            if (angleSign > 0) {
                angle = Degrees.of(45.0);
            } else if (angleSign < 0) {
                angle = Degrees.of(-45.0);
            } else {
                angle = Degrees.of(0.0);
            }
            desiredRotation = Optional.of(
                new Rotation2d(inputSpeedTranslation.getX(), inputSpeedTranslation.getY()).plus(new Rotation2d(angle))
            );
        }

        var assistSpeed = driveAssist();
        var velocityY = assistSpeed.isEmpty() ? inputSpeedTranslation.getY() : assistSpeed.get().vyMetersPerSecond;
        if (desiredRotation.isPresent()) {
            return drivetrain.fieldCentricFacingAngleDrive
                .withVelocityX(inputSpeedTranslation.getX())
                .withVelocityY(velocityY)
                .withTargetDirection(desiredRotation.get());
        }
        return drivetrain.fieldCentricDrive
            .withVelocityX(inputSpeedTranslation.getX())
            .withVelocityY(velocityY)
            .withRotationalRate(inputSpeedRotation);
    }

    private Optional<ChassisSpeeds> driveAssist() {
        if (
            !drivetrain.getDriveFlagValue(DriveFlag.DRIVE_ASSIST) ||
            !DriverStation.isTeleop() ||
            drivetrain.getDriveFlagValue(DriveFlag.MANUAL_ALIGN)
        ) {
            return Optional.empty();
        }

        Pose2d robotPose = drivetrain.getState().Pose;

        // trench assist
        var trenchZone = PoseUtil.getPoseTrenchZone(robotPose);
        if (trenchZone.isPresent()) {
            Translation2d trenchFocus = trenchZone.get().FOCUS;

            var leftExtentDiagonal = Pair.of(
                new Translation2d(
                    DrivetrainConstants.CHASSIS_SIZE_X.div(2).plus(HopperConstants.HOPPER_EXTENT),
                    DrivetrainConstants.BUMPER_SIZE_Y.div(2)
                ),
                new Translation2d(DrivetrainConstants.BUMPER_SIZE_X.div(-2), DrivetrainConstants.BUMPER_SIZE_Y.div(-2))
            );
            var rotatedLeftExtentDiagonal = Pair.of(
                leftExtentDiagonal.getFirst().rotateAround(Translation2d.kZero, robotPose.getRotation()),
                leftExtentDiagonal.getSecond().rotateAround(Translation2d.kZero, robotPose.getRotation())
            );
            var rotatedRightExtentDiagonal = Pair.of(
                new Translation2d(
                    leftExtentDiagonal.getFirst().getX(),
                    -leftExtentDiagonal.getFirst().getY()
                ).rotateAround(Translation2d.kZero, robotPose.getRotation()),
                new Translation2d(
                    leftExtentDiagonal.getSecond().getX(),
                    -leftExtentDiagonal.getSecond().getY()
                ).rotateAround(Translation2d.kZero, robotPose.getRotation())
            );
            double leftDiagonalVertical = Math.abs(
                rotatedLeftExtentDiagonal.getFirst().getY() - rotatedLeftExtentDiagonal.getSecond().getY()
            );
            double rightDiagonalVertical = Math.abs(
                rotatedRightExtentDiagonal.getFirst().getY() - rotatedRightExtentDiagonal.getSecond().getY()
            );
            var mostVerticalDiagonal =
                leftDiagonalVertical > rightDiagonalVertical ? rotatedLeftExtentDiagonal : rotatedRightExtentDiagonal;
            Distance focusOffset = mostVerticalDiagonal
                .getFirst()
                .getMeasureY()
                .plus(mostVerticalDiagonal.getSecond().getMeasureY())
                .div(-2);
            Translation2d alignFocus = trenchFocus.plus(new Translation2d(Meters.of(0.0), focusOffset));

            Distance errorX = alignFocus.getMeasureX().minus(robotPose.getMeasureX());
            Distance errorY = alignFocus.getMeasureY().minus(robotPose.getMeasureY());
            Distance localErrorY =
                trenchFocus.getY() < FieldConstants.ALLIANCE_HEIGHT.baseUnitMagnitude() / 2
                    ? errorY.copy()
                    : errorY.times(-1);

            boolean hasPassed = !errorX.isNear(Meters.zero(), OperatorConstants.TRENCH_ASSIST_PASS_POSITION_TOLERANCE);
            boolean isNotApproaching =
                Math.signum(drivetrain.getInputX(true)) == -Math.signum(errorX.magnitude()) ||
                Math.abs(drivetrain.getInputX(true)) <= OperatorConstants.TRENCH_ASSIST_APPROACH_INPUT_TO_TOLERANCE;
            if (hasPassed && isNotApproaching) {
                return Optional.empty();
            }

            double vy =
                TRENCH_ASSIST_ALIGN_STRENGTH *
                Math.signum(errorY.magnitude()) *
                Math.abs(drivetrain.getInputVelocityX(true));
            double influence = OperatorConstants.TRENCH_ASSIST_ALIGN_INFLUENCE * drivetrain.getInputVelocityY(true);
            boolean aligned =
                localErrorY.baseUnitMagnitude() >=
                    -OperatorConstants.TRENCH_ASSIST_ALIGN_POSITION_INNER_TOLERANCE.baseUnitMagnitude() &&
                localErrorY.baseUnitMagnitude() <=
                    OperatorConstants.TRENCH_ASSIST_ALIGN_POSITION_OUTER_TOLERANCE.baseUnitMagnitude();
            boolean againstAlignment = influence >= Math.abs(vy);
            if (aligned || againstAlignment) {
                vy = 0.0;
            }
            vy += influence;

            return Optional.of(new ChassisSpeeds(0.0, vy, 0.0));
        }

        return Optional.empty();
    }

    private Supplier<SwerveRequest> getDriveSupplier(DriveMode driveMode) {
        return () ->
            switch (driveMode) {
                case TELEOP -> getTeleopDrive();
                case BRAKE -> drivetrain.brakeDrive;
                case POINT -> drivetrain.pointDrive.withModuleDirection(
                    new Rotation2d(drivetrain.getInputX(false), drivetrain.getInputY(false))
                );
                case IDLE -> drivetrain.idleDrive;
            };
    }
}
