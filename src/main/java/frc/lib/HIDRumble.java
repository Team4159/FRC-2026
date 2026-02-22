package frc.lib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

/**
 * rudimentary library for rumble feedback on controllers and HIDs
 */
public class HIDRumble {
    private static final double kDefaultRequestDuration = Units.millisecondsToSeconds(50);
    private static final int kDefaultRequestPriority = 0;
    private static final boolean kRumblePersistWhileDisabled = false;

    private static boolean rumbleEnabled = true;

    private static final HashMap<GenericHID, RumbleManager> rumbleManagerMap = new HashMap<>();

    @SuppressWarnings("unused")
    private static final HIDRumble instance = new HIDRumble();

    private HIDRumble() {
        new SubsystemBase() {
            @Override
            public void periodic() {
                // update all rumble managers
                for (Map.Entry<GenericHID, RumbleManager> rumbleManagerEntry : rumbleManagerMap.entrySet()) {
                    RumbleManager rumbleManager = rumbleManagerEntry.getValue();
                    rumbleManager.update();
                }
            }
        };
    }

    public static void rumble(GenericHID hid, RumbleRequest rumbleRequest) {
        RumbleManager existingRumbleManager = rumbleManagerMap.get(hid);
        RumbleManager rumbleManager = (existingRumbleManager != null) ? existingRumbleManager : new RumbleManager(hid);
        rumbleManager.request(rumbleRequest);
    }

    public static void rumble(CommandGenericHID commandHid, RumbleRequest rumbleRequest) {
        rumble(commandHid.getHID(), rumbleRequest);
    }

    public static void enable(boolean enabled) {
        rumbleEnabled = enabled;
    }

    private static class RumbleManager {
        private final ArrayList<RumbleRequest> rumbleRequestList = new ArrayList<>();
        private int highestPriorityRequestIndex = 0;

        private final GenericHID hid;

        public RumbleManager(GenericHID hid) {
            this.hid = hid;
            HIDRumble.rumbleManagerMap.put(hid, this);
        }

        public void request(RumbleRequest rumbleRequest) {
            rumbleRequestList.add(rumbleRequest);
            if (rumbleRequest.priority > highestPriorityRequestIndex) {
                highestPriorityRequestIndex = rumbleRequest.priority;
            }
        }

        public void update() {
            boolean robotEnabled = DriverStation.isEnabled();

            if (!robotEnabled && !kRumblePersistWhileDisabled) {
                rumbleRequestList.clear();
                highestPriorityRequestIndex = 0;
            } else {
                Iterator<RumbleRequest> removeIterator = rumbleRequestList.iterator();
                boolean removedHighestPriorityRequest = false;
                while (removeIterator.hasNext()) {
                    RumbleRequest rumbleRequest = removeIterator.next();
                    if (rumbleRequest.isExpired()) {
                        removeIterator.remove();
                        if (rumbleRequest.priority == highestPriorityRequestIndex) {
                            removedHighestPriorityRequest = true;
                        }
                    }
                }
                if (removedHighestPriorityRequest) {
                    updateHighestPriorityRequestIndex();
                }
            }

            if (rumbleEnabled && rumbleRequestList.size() > 0) {
                setRumbleFromRequest(getLatestHighestPriorityRequest());
            } else {
                hid.setRumble(RumbleType.kBothRumble, 0);
            }
        }

        private void setRumbleFromRequest(RumbleRequest rumbleRequest) {
            double leftStrength = 0, rightStrength = 0;
            switch (rumbleRequest.rumbleType) {
                case kLeftRumble:
                    leftStrength = rumbleRequest.strength;
                    break;
                case kRightRumble:
                    rightStrength = rumbleRequest.strength;
                    break;
                case kBothRumble:
                    leftStrength = rumbleRequest.strength;
                    rightStrength = rumbleRequest.strength;
                    break;
            }
            hid.setRumble(RumbleType.kLeftRumble, leftStrength);
            hid.setRumble(RumbleType.kRightRumble, rightStrength);
        }

        private void updateHighestPriorityRequestIndex() {
            highestPriorityRequestIndex = 0;
            for (RumbleRequest rumbleRequest : rumbleRequestList) {
                if (rumbleRequest.priority > highestPriorityRequestIndex) {
                    highestPriorityRequestIndex = rumbleRequest.priority;
                }
            }
        }

        private RumbleRequest getLatestHighestPriorityRequest() {
            for (int i = rumbleRequestList.size() - 1; i >= 0; i--) {
                RumbleRequest rumbleRequest = rumbleRequestList.get(i);
                if (rumbleRequest.priority == highestPriorityRequestIndex) {
                    return rumbleRequest;
                }
            }
            return null;
        }
    }

    public static class RumbleRequest {
        public final double start, lifespan, strength;
        public final RumbleType rumbleType;
        public final int priority;

        public RumbleRequest(RumbleType rumbleType, double strength, int priority, double lifespan) {
            this.start = Timer.getFPGATimestamp();
            this.rumbleType = rumbleType;
            this.strength = MathUtil.clamp(strength, 0, 1);
            this.priority = priority;
            this.lifespan = Math.max(0, lifespan);
        }

        public RumbleRequest(RumbleType rumbleType, double strength) {
            this(rumbleType, strength, kDefaultRequestPriority, kDefaultRequestDuration);
        }

        public RumbleRequest(double strength) {
            this(RumbleType.kBothRumble, strength, kDefaultRequestPriority, kDefaultRequestDuration);
        }

        public RumbleRequest(RumbleType rumbleType, double strength, int priority) {
            this(rumbleType, strength, priority, kDefaultRequestDuration);
        }

        public RumbleRequest(double strength, int priority) {
            this(RumbleType.kBothRumble, strength, priority, kDefaultRequestDuration);
        }

        public boolean isExpired() {
            return Timer.getFPGATimestamp() - start > lifespan;
        }
    }
}
