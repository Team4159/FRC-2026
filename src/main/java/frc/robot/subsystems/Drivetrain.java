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

    //I hate this implementation but I am stupid
    //false = regular auto path mode
    //true = auto aim auto path mode
    private boolean autoPathAutoAimMode = false;

    //for auto aim in auto
    private AutoAim autoAimCommand ;

    /**
     * 
     * @param controller a CommandXboxController used to control the robot in teleop
     * @param leds LEDs subsystem for the autoaim command
     */
    public Drivetrain(CommandXboxController controller, LEDs leds) {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        this.controller = controller;
        autoAimCommand = new AutoAim(this, leds, true);
    }

    public double getInputX() {
        double rawInput = -controller.getLeftY();
        double filteredInput = MathUtil.applyDeadband(OperatorConstants.kDriverControllerVelocityDeadband, rawInput);
        return Math.abs(Math.pow(filteredInput, 2)) * Math.signum(filteredInput);
    }

    public double getInputY() {
        double rawInput = -controller.getLeftX();
        double filteredInput = MathUtil.applyDeadband(OperatorConstants.kDriverControllerVelocityDeadband, rawInput);
        return Math.abs(Math.pow(filteredInput, 2)) * Math.signum(filteredInput);
    }

    public double getInputRotation() {
        double rawInput = -controller.getRightX();
        double filteredInput = MathUtil.applyDeadband(OperatorConstants.kDriverControllerVelocityDeadband, rawInput);
        return Math.abs(Math.pow(filteredInput, 2)) * Math.signum(filteredInput);
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
     * omega is calculated differently depending on the mode. if autoPathAutoAimMode is true it will use the omega from the auto aim command to aim at the hub, otherwise it will use target speeds and PID
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
        //if autoPathAutoAimMode is true it will use the omega from the auto aim command to aim at the hub
        //otherwise it will use target speeds and PID
        if(autoPathAutoAimMode){
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
