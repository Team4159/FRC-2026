package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command{
    private final Shooter shooter;
    private final Shooter hood;
    
    public ShooterCommand(Shooter shooter, Shooter hood) {
        this.shooter = shooter;
        this.hood = hood;
        addRequirements(shooter, hood);
    }

    public void execute() {
        shooter.shootSpeed(10);

        
    }
}
