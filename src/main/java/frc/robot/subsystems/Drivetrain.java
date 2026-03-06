package frc.robot.subsystems;

import static frc.robot.Constants.OperatorConstants.*;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.FieldUtil;
import frc.lib.HIDRumble;
import frc.lib.HIDRumble.RumbleRequest;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.DriveMode;
import frc.robot.commands.AutoAim;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class Drivetrain extends CommandSwerveDrivetrain {

    public final Pigeon2 pigeon = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);

    public final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(kAimKP, kAimKI, kAimKD)
            .withTargetRateFeedforward(kAimFeedForward);
    public final SwerveRequest.RobotCentricFacingAngle robotCentricFacingAngleDrive = new SwerveRequest.RobotCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(kAimKP, kAimKI, kAimKD)
            .withTargetRateFeedforward(kAimFeedForward);
    public final SwerveRequest.FieldCentricFacingAngle alignFacingAngleDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(kAimKP, kAimKI, kAimKD)
            .withTargetRateFeedforward(kAimFeedForward);
    public final SwerveRequest.SwerveDriveBrake brakeDrive = new SwerveRequest.SwerveDriveBrake();
    public final SwerveRequest.PointWheelsAt pointDrive = new SwerveRequest.PointWheelsAt();
    public final SwerveRequest.Idle idleDrive = new SwerveRequest.Idle();

    public final Trigger crashTrigger = new Trigger(
            () -> Math.hypot(pigeon.getAccelerationX().getValue().in(MetersPerSecondPerSecond),
                    pigeon.getAccelerationY().getValue().in(MetersPerSecondPerSecond)) >= FieldConstants.g * 2);

    private final Supplier<Double> inputX;
    private final Supplier<Double> inputY;
    private final Supplier<Double> inputRotationVelocity;
    private final Supplier<Double> inputRotationX;
    private final Supplier<Double> inputRotationY;

    private boolean reduceSpeedEnabled = false;
    private boolean driveAssistEnabled = true;
    
    private boolean autoPathAutoAimMode = false;
    private AutoAim autoAimCommand;
    

    private Optional<Rotation2d> desiredRotation = Optional.empty();

    public Drivetrain(CommandXboxController controller) {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight,
                TunerConstants.BackLeft, TunerConstants.BackRight);
        this.inputX = () -> -controller.getLeftY();
        this.inputY = () -> -controller.getLeftX();
        this.inputRotationVelocity = () -> -controller.getRightX();
        this.inputRotationX = () -> -controller.getRightY();
        this.inputRotationY = () -> -controller.getRightX();

        controller.a().onTrue(Commands
                .runOnce(() -> HIDRumble.rumble(controller.getHID(),
                        new RumbleRequest(RumbleType.kRightRumble, 1, 0.25, 1))));
    }

    public class Drive extends Command {
        private final BooleanSupplier robotRelativeSupplier;
        private final Supplier<SwerveRequest> driveSupplier;

        public Drive(DriveMode driveMode, BooleanSupplier robotRelativeSupplier) {
            this.robotRelativeSupplier = robotRelativeSupplier;
            this.driveSupplier = getDriveSupplier(driveMode);
            addRequirements(Drivetrain.this);
        }

        public Drive(DriveMode mode) {
            this(mode, () -> false);
        }

        @Override
        public void execute() {
            System.out.println("driveCommand");
            setControl(driveSupplier.get());
        }

        private double getMaxTranslationSpeed() {
            return kMaxTranslationSpeed * (reduceSpeedEnabled ? kPrimaryReduceSpeedTranslationFactor : 1);
        }

        private double getMaxRotationSpeed() {
            return kMaxRotationSpeed * (reduceSpeedEnabled ? kPrimaryReduceSpeedRotationFactor : 1);
        }

        private double getSpeedX() {
            return getInputX() * getMaxTranslationSpeed();
        }

        private double getSpeedY() {
            return getInputY() * getMaxTranslationSpeed();
        }

        private double getSpeedRotation() {
            return getInputRotationVelocity() * getMaxRotationSpeed();
        }

        private Optional<ChassisSpeeds> driveAssist() {
            if (!driveAssistEnabled || !DriverStation.isTeleop()) {
                return Optional.empty();
            }
            Pose2d robotPose = getState().Pose;

            // trench assist
            var trenchZone = FieldUtil.getPoseTrenchZone(robotPose);
            if (trenchZone.isPresent()) {
                Distance errorX = trenchZone.get().x.minus(robotPose.getMeasureX());
                Distance errorY = trenchZone.get().y.minus(robotPose.getMeasureY());

                boolean hasPassed = !errorX.isNear(Meters.zero(), OperatorConstants.kTrenchAssistPassPositionTolerance);
                boolean isNotApproaching = (Math.signum(getInputX()) == -Math.signum(errorX.magnitude()))
                        || Math.abs(getInputX()) <= OperatorConstants.kTrenchAssistApproachInputTolerance;
                if (hasPassed && isNotApproaching) {
                    return Optional.empty();
                }

                double vy = kTrenchAssistAlignStrength * getMaxTranslationSpeed() * Math.signum(errorY.magnitude())
                        * Math.abs(getInputX());
                double influence = OperatorConstants.kTrenchAssistAlignInfluence * getSpeedY();
                boolean insideTolerance = errorY.isNear(Meters.zero(),
                        OperatorConstants.kTrenchAssistAlignPositionTolerance);
                boolean againstAlignment = (influence >= Math.abs(vy));
                if (insideTolerance || againstAlignment) {
                    vy = 0.0;
                }
                vy += influence;

                return Optional.of(new ChassisSpeeds(
                        0.0,
                        vy,
                        0.0));
            }

            return Optional.empty();
        }

        private Supplier<SwerveRequest> getDriveSupplier(DriveMode driveMode) {
            return () -> switch (driveMode) {
                case FREE -> {
                    if (isInputIdle()) {
                        if (desiredRotation.isPresent()) {
                            if (Math.abs(Math
                                    .abs(getState().Pose.getRotation().minus(desiredRotation.get())
                                            .getDegrees())) < 1.0) {
                                yield brakeDrive;
                            }
                        } else {
                            yield brakeDrive;
                        }
                    }
                    if (robotRelativeSupplier.getAsBoolean()) {
                        if (desiredRotation.isPresent()) {
                            yield robotCentricFacingAngleDrive.withVelocityX(getSpeedX())
                                    .withVelocityY(getSpeedY())
                                    .withTargetDirection(desiredRotation.get());
                        }
                        yield robotCentricDrive.withVelocityX(getSpeedX())
                                .withVelocityY(getSpeedY())
                                .withRotationalRate(getSpeedRotation());
                    }
                    var assistSpeed = driveAssist();
                    var velocityY = assistSpeed.isEmpty() ? getSpeedY()
                            : assistSpeed.get().vyMetersPerSecond;
                    if (desiredRotation.isPresent()) {
                        yield fieldCentricFacingAngleDrive.withVelocityX(getSpeedX())
                                .withVelocityY(velocityY)
                                .withTargetDirection(desiredRotation.get());
                    }
                    yield fieldCentricDrive.withVelocityX(getSpeedX())
                            .withVelocityY(velocityY)
                            .withRotationalRate(getSpeedRotation());
                }
                case BRAKE -> brakeDrive;
                case POINT -> pointDrive
                        .withModuleDirection(new Rotation2d(getInputX(), getInputY()));
                case IDLE -> idleDrive;
                case MANUAL_ALIGN -> {
                    Translation2d rawInput = getRawInputTranslation();
                    double x = Math.signum(MathUtil.applyDeadband(rawInput.getX(), kPrimaryAlignModeDeadband, 1));
                    double y = Math.signum(MathUtil.applyDeadband(rawInput.getY(), kPrimaryAlignModeDeadband, 1));

                    if (x != 0 && y != 0) {
                        x /= Math.sqrt(2);
                        y /= Math.sqrt(2);
                    }

                    double velocityX = kPrimaryAlignModeSpeedTranslationFactor * x * getMaxTranslationSpeed();
                    double velocityY = kPrimaryAlignModeSpeedTranslationFactor * y * getMaxTranslationSpeed();
                    double rotationalRate = kPrimaryAlignModeSpeedRotationFactor * kMaxRotationSpeed
                            * getInputRotationVelocity();

                    if (robotRelativeSupplier.getAsBoolean()) {
                        yield robotCentricDrive.withVelocityX(velocityX)
                                .withVelocityY(velocityY)
                                .withRotationalRate(rotationalRate);
                    }
                    yield fieldCentricDrive.withVelocityX(velocityX)
                            .withVelocityY(velocityY)
                            .withRotationalRate(rotationalRate);
                }
                case INTAKE -> {
                    // TODO: decide whats the optimal deadzone or have some heuristic to check if
                    // the joystick was released
                    if (Math.hypot(getInputX(), getInputY()) >= 0.2) {
                        yield fieldCentricFacingAngleDrive
                                .withVelocityX(getSpeedX())
                                .withVelocityY(getSpeedY())
                                .withTargetDirection(new Rotation2d(getInputX(), getInputY()));
                    }
                    yield fieldCentricDrive
                            .withVelocityX(getSpeedX())
                            .withVelocityY(getSpeedY());
                }
                case RADIAL -> {
                    if (isInputIdle()) {
                        yield brakeDrive;
                    }
                    double radialInput = MathUtil.applyDeadband(inputX.get(), kPrimaryRadialModeDeadband, 1);
                    double tangentialInput = MathUtil.applyDeadband(inputY.get(), kPrimaryRadialModeDeadband, 1);
                    Pose2d robotPose = getState().Pose;
                    Pose2d hubPose = FieldConstants.hubLocations.get(DriverStation.getAlliance().orElse(Alliance.Blue));
                    Translation2d radialVector = new Translation2d(
                            hubPose.getX() - robotPose.getX(),
                            hubPose.getY() - robotPose.getY());
                    if (radialVector.getNorm() <= 1e-3) {
                        radialVector = new Translation2d(0.0, 0.0);
                    } else {
                        radialVector = radialVector.div(radialVector.getNorm());
                    }
                    Translation2d tangentialVector = new Translation2d(
                            radialVector.getY(),
                            radialVector.getX());
                    double velocityX = getMaxTranslationSpeed()
                            * (radialInput * radialVector.getX() - tangentialInput * tangentialVector.getX());
                    double velocityY = getMaxTranslationSpeed()
                            * (radialInput * radialVector.getY() + tangentialInput * tangentialVector.getY());
                    double velocityMagnitude = Math.hypot(velocityX, velocityY);
                    if (velocityMagnitude > getMaxTranslationSpeed()) {
                        double correctingFactor = getMaxTranslationSpeed() / velocityMagnitude;
                        velocityX *= correctingFactor;
                        velocityY *= correctingFactor;
                    }
                    if (desiredRotation.isPresent()) {
                        yield fieldCentricFacingAngleDrive
                                .withVelocityX(velocityX)
                                .withVelocityY(velocityY)
                                .withTargetDirection(desiredRotation.get());
                    }
                    yield fieldCentricDrive
                            .withVelocityX(velocityX)
                            .withVelocityY(velocityY);
                }
            };
        }
    }

    public void enableReduceSpeed(boolean enabled) {
        this.reduceSpeedEnabled = enabled;
    }

    public void enableDriveAssist(boolean enabled) {
        this.driveAssistEnabled = enabled;
    }

    public void setDesiredRotation(Rotation2d desiredRotation) {
        this.desiredRotation = Optional.of(desiredRotation);
    }

    public void clearDesiredRotation() {
        this.desiredRotation = Optional.empty();
    }

    /**
     * @return the field relative translation input (-left joystick y input,
     *         -left
     *         joystick x input), from magnitude range -1 to 1. no deadzone is
     *         applied
     */
    public Translation2d getRawInputTranslation() {
        Translation2d rawInput = new Translation2d(inputX.get(), inputY.get());
        if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
            rawInput = rawInput.times(-1);
        }
        return rawInput;
    }

    /**
     * @return the field relative translation input (-left joystick y input,
     *         -left
     *         joystick x input), from magnitude range -1 to 1. a deadzone is
     *         applied.
     */
    public Translation2d getInputTranslation() {
        Translation2d rawInput = getRawInputTranslation();
        Vector<N2> filteredInputVector = rawInput.toVector();
        filteredInputVector = MathUtil.applyDeadband(filteredInputVector, kPrimaryTranslationDeadband, 1);

        // apply max radius
        filteredInputVector = filteredInputVector.div(kPrimaryTranslationRadius);

        // apply exponent
        if (filteredInputVector.norm() > 0.0) {
            filteredInputVector = filteredInputVector.unit().times(Math.pow(filteredInputVector.norm(), kPrimaryTranslationExponent));
        }

        // clamp values
        if (filteredInputVector.norm() > 1) {
            filteredInputVector = filteredInputVector.div(filteredInputVector.norm());
        }

        return new Translation2d(filteredInputVector);
    }

    /**
     * @return the field relative x input (-left joystick y input), from range -1
     *         to
     *         1. a deadzone and quadratic are applied for better control.
     */
    public double getInputX() {
        return getInputTranslation().getX();
    }

    /**
     * @return the field relative y input (-left joystick x input), from range -1
     *         to
     *         1. a deadzone and quadratic are applied for better control.
     */
    public double getInputY() {
        return getInputTranslation().getY();
    }

    /**
     * @return the field relative rotation input (-right joystick x), from range -1
     *         to 1. no deadzone is applied
     */
    public double getRawInputRotationVelocity() {
        return inputRotationVelocity.get();
    }

    /**
     * @return the field relative rotation input (-right joystick x), from range -1
     *         to 1. a deadzone and quadratic are applied for better control.
     */
    public double getInputRotationVelocity() {
        double rawInput = getRawInputRotationVelocity();
        double filteredInput = MathUtil.applyDeadband(Math.abs(rawInput),
                kPrimaryRotationDeadband, 1);
        return Math.abs(Math.pow(filteredInput, kPrimaryRotationExponent)) * Math.signum(rawInput);
    }

    public Rotation2d getRawInputRotation() {
        return new Rotation2d(inputRotationX.get(), inputRotationY.get());
    }

    public Optional<Rotation2d> getInputRotation() {
        if (Math.hypot(inputRotationX.get(), inputRotationY.get()) < kPrimaryRotationDeadband) {
            return Optional.empty();
        }
        Rotation2d filteredInput = getRawInputRotation();
        if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
            filteredInput = filteredInput.plus(Rotation2d.k180deg);
        }
        return Optional.of(filteredInput);
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }
    
    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
            () -> getState().Pose,
            this::resetPose,
            this::followPath,
            true,
            this,
            trajLogger
        );
    }

    /** set the AutoAim command to be used for shooting while moving during auto */
    public void setAutonomousAutoAimCommand(AutoAim autoAimCommand){
        this.autoAimCommand = autoAimCommand;
    }

    /**
     * Follows the given field-centric path sample with PID.
     * 
     * if autoPathAutoAimMode is true and the setAutonomousAutoAimCommand() method was used to set the autoAimCommand, it will use the omega from the auto aim command to aim at the hub
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        //current robot pose
        var pose = getState().Pose;
        //choreo calculated target speeds (time based)
        var targetSpeeds = sample.getChassisSpeeds();
        //calculate translation velocities with target speeds and PID to correct for error
        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(
            pose.getY(), sample.y
        );

        //omega is calculated differently depending on the mode
        //if autoPathAutoAimMode is true and the setAutonomousAutoAimCommand() method was used to set the autoAimCommand, it will use the omega from the auto aim command to aim at the hub
        //otherwise it will use target speeds and PID
        if(autoPathAutoAimMode && autoAimCommand != null){
            //get the desired omega directly from the auto aim controller(instead of calculated speeds and PID)
            targetSpeeds.omegaRadiansPerSecond = autoAimCommand.getDesiredOmega();
        }
        else{
            m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
            //get desired omega from calculated speeds and PID
            targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
                pose.getRotation().getRadians(), sample.heading
            );
        }
        //send the calculated speeds to the drivetrain
        setControl(
                m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
                    .withWheelForceFeedforwardsX(sample.moduleForcesX())
                    .withWheelForceFeedforwardsY(sample.moduleForcesY())
            );
    }

    /** @param autoPathAutoAimMode if true the robot will run autoaim along the auto trajectory 
     * a value of true will activate the AutoAim command and a value of false will cancel it. it will also schedule and cancel the auto aim command object stored in the Drivetrain class.
     * I hate this implementation but I have negative intelligence
    */
    public void setAutoPathAutoAimMode(boolean autoPathAutoAimMode){
        this.autoPathAutoAimMode = autoPathAutoAimMode;
        if(autoPathAutoAimMode){
            CommandScheduler.getInstance().schedule(autoAimCommand);
        }
        else{
            CommandScheduler.getInstance().cancel(autoAimCommand);
        }
    }

    /**
     * @return {@Code true} if there is no joystick input
     */
    public boolean isInputIdle() {
        boolean noInputRotation = true;
        if (desiredRotation.isPresent()) {
            noInputRotation = getInputRotation().isEmpty();
        }
        return getInputTranslation().getNorm() == 0.0 && getInputRotationVelocity() == 0.0 && noInputRotation;
    }

}
