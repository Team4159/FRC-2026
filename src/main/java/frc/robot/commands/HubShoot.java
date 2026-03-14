package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.HopperConstants.HopperState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class HubShoot extends Command{
    private final Shooter shooter;
    private final Intake intake;
    private final Hopper hopper;

    public HubShoot(Shooter shooter, Intake intake, Hopper hopper){
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;

        addRequirements(shooter, hopper);
    }

    @Override
    public void initialize(){
        shooter.setSpeed(ShooterConstants.hubAngularVelocity);
        shooter.adjustTrajectoryAngle(ShooterConstants.hubHoodPitch);
        CommandScheduler.getInstance().schedule(intake.new BounceIntake());
    }

    @Override
    public void execute(){
        if(shooter.isAtPitch() && shooter.isAtSpeed()){
            shooter.setFeederSpeed(FeederState.FEED.percentage);
            hopper.setHopperSpeed(HopperState.FEED.percentage);
        }
        else{
            shooter.setFeederSpeed(FeederState.STOP.percentage);
            hopper.setHopperSpeed(HopperState.STOP.percentage);
        }
    }

    @Override
    public void end(boolean interrupted){
        shooter.setSpeed(ShooterConstants.restingAngularVelocity);
        shooter.adjustHood(Degrees.of(2));
        shooter.setFeederSpeed(FeederState.STOP.percentage);
        hopper.setHopperSpeed(HopperState.STOP.percentage);
        CommandScheduler.getInstance().schedule(intake.new ChangeStates(IntakeState.DOWN_OFF));
    }
}