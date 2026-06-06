package frc.lib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class FluentTrigger {

    public static class Builder {

        private final Map<TriggerState, Command> stateCommandMap = new HashMap<>();
        private Command defaultCommand;

        public Builder() {}

        public FluentTrigger build() {
            return new FluentTrigger(stateCommandMap, defaultCommand);
        }

        public Builder defaultCommand(Command defaultCommand) {
            if (this.defaultCommand != null) {
                throw new IllegalStateException("FluentTrigger default command already set");
            }
            this.defaultCommand = defaultCommand;
            return this;
        }

        public Builder bind(int priority, Trigger trigger, Command command) {
            TriggerState state = new TriggerState(trigger, priority);
            stateCommandMap.put(state, command);
            return this;
        }

        public Builder bind(Trigger trigger, Command command) {
            return bind(0, trigger, command);
        }
    }

    private static class TriggerState {

        private final Trigger trigger;
        private final int priority;

        public TriggerState(Trigger trigger, int priority) {
            this.trigger = trigger;
            this.priority = priority;
        }
    }

    // the most recently appended state is prioritized first
    private final ArrayList<TriggerState> stateList = new ArrayList<>();
    private final Map<TriggerState, Command> stateCommandMap;

    private final Command defaultCommand;
    private Command activeCommand;

    private FluentTrigger(Map<TriggerState, Command> stateCommandMap, Command defaultCommand) {
        this.stateCommandMap = stateCommandMap;
        this.defaultCommand = defaultCommand;

        this.activeCommand = defaultCommand;
        updateState();
        stateCommandMap.forEach((state, command) -> {
            state.trigger.onTrue(new InstantCommand(() -> addQueue(state)));
            state.trigger.onFalse(new InstantCommand(() -> removeQueue(state)));
        });
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
        activeCommand = stateCommandMap.get(nextState);

        boolean activeCommandChanged = (activeCommand != oldActiveCommand);
        if (activeCommandChanged && oldActiveCommand != null) {
            CommandScheduler.getInstance().cancel(oldActiveCommand);
        }

        if (!activeCommand.isScheduled()) {
            CommandScheduler.getInstance().schedule(activeCommand);
        }
    }
}
