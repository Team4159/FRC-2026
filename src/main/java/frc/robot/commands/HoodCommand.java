package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class HoodCommand extends Command{
    private final Shooter hood;

    public HoodCommand(Shooter hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        //hood.adjustHood(angle); // fix
    }
    @Override
    public void end(boolean interupt) {
        hood.adjustHood(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
