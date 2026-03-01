package frc.lib;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FluentTrigger {
    private record CommandBind(Trigger trigger, Command command) {
    }

    private Command activeCommand;
    private Command defaultCommand;
    private final ArrayList<Integer> activeStateQueue = new ArrayList<Integer>();
    private final ArrayList<CommandBind> triggerCommandBindList = new ArrayList<CommandBind>();

    private FluentTrigger() {
    }

    public static FluentTrigger build() {
        return new FluentTrigger();
    }


    public FluentTrigger setDefault(Command defaultCommand) {
        if (this.defaultCommand != null) {
            throw new IllegalStateException("FluentTrigger default command already set");
        }
        this.defaultCommand = defaultCommand;
        this.activeCommand = defaultCommand;
        updateState();
        return this;
    }

    public FluentTrigger bind(Trigger trigger, Command command) {
        CommandBind triggerCommandBind = new CommandBind(trigger, command);
        int state = triggerCommandBindList.size();
        triggerCommandBind.trigger.onTrue(new InstantCommand(() -> addQueue(state)));
        triggerCommandBind.trigger.onFalse(new InstantCommand(() -> removeQueue(state)));
        triggerCommandBindList.add(triggerCommandBind);
        return this;
    }

    private void addQueue(int state) {
        if (activeStateQueue.indexOf(state) >= 0) {
            return;
        }
        activeStateQueue.add(state);
        updateState();
    }

    private void removeQueue(int state) {
        int index = activeStateQueue.indexOf(state);
        if (index < 0) {
            return;
        }
        activeStateQueue.remove(index);
        updateState();
    }

    private void updateState() {
        if (activeStateQueue.size() == 0) {
            if (activeCommand != null && activeCommand.isScheduled()) {
                activeCommand.cancel();
            }
            if (defaultCommand != null && !defaultCommand.isScheduled()) {
                CommandScheduler.getInstance().schedule(defaultCommand);
                activeCommand = defaultCommand;
            }
            return;
        }

        int activeState = activeStateQueue.get(activeStateQueue.size() - 1);
        Command oldActiveCommand = activeCommand;
        activeCommand = triggerCommandBindList.get(activeState).command;
        boolean activeCommandChanged = (activeCommand != oldActiveCommand);
        if (activeCommandChanged && oldActiveCommand != null) {
            oldActiveCommand.cancel();
        }
        if (!activeCommand.isScheduled()) {
            CommandScheduler.getInstance().schedule(activeCommand);
        }
    }
}