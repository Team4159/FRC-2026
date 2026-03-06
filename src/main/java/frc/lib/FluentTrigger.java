package frc.lib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FluentTrigger {
    private record CommandBind(Trigger trigger, Command command, int priority) {
    }

    private class TriggerState {
        private final int priority;

        public TriggerState(int priority) {
            this.priority = priority;
        }
    }

    // the most recently appended state is prioritized first
    private final ArrayList<TriggerState> stateList = new ArrayList<>();
    private final Map<TriggerState, CommandBind> stateCommandMap = new HashMap<>();

    private Command activeCommand;
    private Command defaultCommand;

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

    public FluentTrigger bind(int priority, Trigger trigger, Command command) {
        TriggerState state = new TriggerState(priority);
        CommandBind triggerCommandBind = new CommandBind(trigger, command, priority);
        triggerCommandBind.trigger.onTrue(new InstantCommand(() -> addQueue(state)));
        triggerCommandBind.trigger.onFalse(new InstantCommand(() -> removeQueue(state)));
        stateCommandMap.put(state, triggerCommandBind);
        return this;
    }

    public FluentTrigger bind(Trigger trigger, Command command) {
        return bind(0, trigger, command);
    }

    private void addQueue(TriggerState state) {
        if (stateList.contains(state)) {
            return;
        }
        stateList.add(state);
        updateState();
    }

    private void removeQueue(TriggerState state) {
        if (!stateList.contains(state)) {
            return;
        }
        stateList.remove(state);
        updateState();
    }

    private void updateState() {
        if (stateList.size() == 0) {
            if (activeCommand != null && activeCommand.isScheduled()) {
                CommandScheduler.getInstance().cancel(activeCommand);
            }
            if (defaultCommand != null && !defaultCommand.isScheduled()) {
                CommandScheduler.getInstance().schedule(defaultCommand);
                activeCommand = defaultCommand;
            }
            return;
        }

        TriggerState nextState = stateList.get(stateList.size() - 1);
        for (int i = stateList.size() - 2; i >= 0; i--) {
            TriggerState nextStateCandidate = stateList.get(i);
            if (nextStateCandidate.priority <= nextState.priority) {
                continue;
            }
            nextState = nextStateCandidate;
        }

        Command oldActiveCommand = activeCommand;
        activeCommand = stateCommandMap.get(nextState).command;

        boolean activeCommandChanged = (activeCommand != oldActiveCommand);
        if (activeCommandChanged && oldActiveCommand != null) {
            CommandScheduler.getInstance().cancel(oldActiveCommand);
        }

        if (!activeCommand.isScheduled()) {
            CommandScheduler.getInstance().schedule(activeCommand);
        }
    }
}