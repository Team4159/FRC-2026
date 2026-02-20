package frc.robot.subsystems;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAim;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class Drivetrain extends CommandSwerveDrivetrain {
    private final CommandXboxController controller;

    /** I hate this implementation but I am stupid
    false = regular auto path mode
    true = auto aim auto path mode*/
    private boolean autoPathAutoAimMode = false;

    /** AutoAim command used during autonomous if autoPathAutoAimMode is true. Must use the setAutonomousAutoAimCommand() method for it to be used in auto.*/
    private AutoAim autoAimCommand;

    /**
     * @param controller a CommandXboxController used to control the robot in teleop
     */
    public Drivetrain(CommandXboxController controller) {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        this.controller = controller;
    }

    /** @param autoAimCommand set the auto aim command that will be used during auto */
    public void setAutonomousAutoAimCommand(AutoAim autoAimCommand){
        this.autoAimCommand = autoAimCommand;
    }

    /** @return the field relative x input (-left joystick y input), from range -1 to 1. a deadzone and quadratic are applied for better control.*/
    public double getInputX() {
        double rawInput = -controller.getLeftY();
        double filteredInput = MathUtil.applyDeadband(rawInput, OperatorConstants.kDriverControllerTranslationDeadband);
        return Math.abs(Math.pow(filteredInput, 2)) * Math.signum(rawInput);
    }

    /** @return the field relative y input (-left joystick x input), from range -1 to 1. a deadzone and quadratic are applied for better control.*/
    /** @return the field relative y input (-left joystick x input), from range -1 to 1. a deadzone and quadratic are applied for better control.*/
    public double getInputY() {
        double rawInput = -controller.getLeftX();
        double filteredInput = MathUtil.applyDeadband(rawInput, OperatorConstants.kDriverControllerTranslationDeadband);
        return Math.abs(Math.pow(filteredInput, 2)) * Math.signum(rawInput);
    }

    /** @return the field relative rotation input (-right joystick x), from range -1 to 1. a deadzone and quadratic are applied for better control.*/
    public double getInputRotation() {
        double rawInput = -controller.getRightX();
        double filteredInput = MathUtil.applyDeadband(rawInput, OperatorConstants.kDriverControllerRotationDeadband);
        return Math.abs(Math.pow(filteredInput, 2)) * Math.signum(rawInput);
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
}
