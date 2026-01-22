package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command{
    private final Shooter shooter;
    
    public ShooterCommand(Shooter shooter, Shooter hood) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    
    @Override
    public void execute() {
        shooter.setSpeed(10);
    }

    @Override
    public void end(boolean interupt) {
        shooter.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
