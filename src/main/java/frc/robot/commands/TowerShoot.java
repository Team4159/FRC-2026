package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.shooter.FeederConstants.FeederState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterSetpoint;

public class TowerShoot extends Command {

    private final Shooter shooter;
    private final Intake intake;
    private final Hopper hopper;

    private Timer timer;
    private boolean feedFlag = false;

    public TowerShoot(Shooter shooter, Intake intake, Hopper hopper) {
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;

        this.timer = new Timer();

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setFlywheelMotorVelocity(ShooterSetpoint.FROM_TOWER);
        shooter.setHoodPitchComplement(ShooterSetpoint.FROM_TOWER);
        CommandScheduler.getInstance().schedule(intake.new BounceIntake());

        timer.reset();
        feedFlag = false;
    }

    @Override
    public void execute() {
        // if (!timer.hasElapsed(ShooterConstants.backwardsTime)){
        //     //run neck backwards if at the beginning
        //     shooter.setFeederSpeed(FeederState.UNSTUCKFEEDER.percentage);
        //     hopper.setHopperSpeed(HopperState.STOP.percentage);
        // }
        if (shooter.isAtHoodPitch() && shooter.isAtFlywheelVelocity()) {
            feedFlag = true;
        }
        if (feedFlag) {
            //shoot the fuel if at the right pitch
            shooter.setFeederDutyCycle(FeederState.FEED.dutyCycle);
            hopper.setDutyCycle(HopperState.FEED.dutyCycle);
        } else {
            //otherwise just wait
            shooter.setFeederDutyCycle(FeederState.STOP.dutyCycle);
            hopper.setDutyCycle(HopperState.STOP.dutyCycle);
        }

        SmartDashboard.putBoolean("isAtPitch", shooter.isAtHoodPitch());
        SmartDashboard.putBoolean("isAtVelocity", shooter.isAtFlywheelVelocity());
    }

    @Override
    public void end(boolean interrupted) {
        //shooter.setSpeed(ShooterConstants.restingAngularVelocity);
        shooter.restFlywheel();
        shooter.restHood();
        //shooter.setFeederSpeed(FeederState.STOP.percentage);
        //hopper.setHopperSpeed(HopperState.STOP.percentage);
        CommandScheduler.getInstance().schedule(intake.new ChangeStates(IntakeState.BOUNCE_UP));
    }
}
