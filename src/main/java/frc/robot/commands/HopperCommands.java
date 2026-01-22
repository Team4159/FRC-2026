package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Hopper;

public class HopperCommands extends Command{

    
    private final Hopper hopper;

    public HopperCommands(Hopper hopper){
        this.hopper = hopper;
        addRequirements(hopper);
    }

    @Override
    public void execute(){
        hopper.setHopperSpeed(0.5);
    }

    @Override
    public void end(boolean interupt){
        hopper.stop()
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}