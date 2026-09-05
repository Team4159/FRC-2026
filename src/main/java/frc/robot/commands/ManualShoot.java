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
        shooter.setVelocity(ShooterConstants.SHOOTER_ANGULAR_VELOCITY);
    }

    @Override
    public void execute() {
        if (shooter.isAtVelocity()) {
            shooter.setFeederDutyCycle(FeederState.FEED.dutyCycle);
            hopper.setHopperDutyCycle(HopperState.FEED.dutyCycle);
        } else {
            shooter.setFeederDutyCycle(FeederState.STOP.dutyCycle);
            hopper.setHopperDutyCycle(HopperState.STOP.dutyCycle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFeederDutyCycle(FeederState.STOP.dutyCycle);
        hopper.setHopperDutyCycle(HopperState.STOP.dutyCycle);
    }
}
