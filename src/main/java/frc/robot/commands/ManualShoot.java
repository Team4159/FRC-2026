package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.HopperConstants.HopperState;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class ManualShoot extends Command {

    private final Shooter shooter;
    private final Hopper hopper;

    public ManualShoot(Shooter shooter, Hopper hopper) {
        this.shooter = shooter;
        this.hopper = hopper;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setSpeed(ShooterConstants.SHOOTER_ANGULAR_VELOCITY);
    }

    @Override
    public void execute() {
        if (shooter.isAtSpeed()) {
            shooter.setFeederSpeed(FeederState.FEED.percentage);
            hopper.setHopperSpeed(HopperState.FEED.percentage);
        } else {
            shooter.setFeederSpeed(FeederState.STOP.percentage);
            hopper.setHopperSpeed(HopperState.STOP.percentage);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFeederSpeed(FeederState.STOP.percentage);
        hopper.setHopperSpeed(HopperState.STOP.percentage);
    }
}
