package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.HopperConstants.HopperState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TowerShoot extends Command{
    private final Shooter shooter;
    private final Intake intake;
    private final Hopper hopper;

    private Timer timer;

    public TowerShoot(Shooter shooter, Intake intake, Hopper hopper){
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;

        this.timer = new Timer();

        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        shooter.setSpeed(ShooterConstants.towerAngularVelocity);
        shooter.adjustTrajectoryAngle(ShooterConstants.towerHoodPitch);
        CommandScheduler.getInstance().schedule(intake.new BounceIntake());

        timer.reset();
    }

    @Override
    public void execute(){
        // if (!timer.hasElapsed(ShooterConstants.backwardsTime)){
        //     //run neck backwards if at the beginning
        //     shooter.setFeederSpeed(FeederState.UNSTUCKFEEDER.percentage);
        //     hopper.setHopperSpeed(HopperState.STOP.percentage);
        // }
        // if (shooter.isAtPitch() && shooter.isAtSpeed()) {
        //     //shoot the fuel if at the right pitch
        //     shooter.setFeederSpeed(FeederState.FEED.percentage);
        //     hopper.setHopperSpeed(HopperState.FEED.percentage);
        // } else {
        //     //otherwise just wait
        //     shooter.setFeederSpeed(FeederState.STOP.percentage);
        //     hopper.setHopperSpeed(HopperState.STOP.percentage);
        // }

        SmartDashboard.putBoolean("isAtPitch", shooter.isAtPitch());
        SmartDashboard.putBoolean("isatspeed", shooter.isAtSpeed());
    }

    @Override
    public void end(boolean interrupted){
        //shooter.setSpeed(ShooterConstants.restingAngularVelocity);
        shooter.stopShooter();
        shooter.adjustHood(Degrees.of(2));
        //shooter.setFeederSpeed(FeederState.STOP.percentage);
        //hopper.setHopperSpeed(HopperState.STOP.percentage);
        CommandScheduler.getInstance().schedule(intake.new ChangeStates(IntakeState.DOWN_OFF));
    }
}