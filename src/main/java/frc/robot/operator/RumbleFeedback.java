package frc.robot.operator;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.lib.HIDRumble;
import frc.lib.HIDRumble.RumbleRequest;

public class RumbleFeedback {

    public static void zero(GenericHID hid) {
        HIDRumble.rumble(hid, new RumbleRequest(RumbleType.kRightRumble, 0.5, 0.25));
    }

    public static void toggleDriveAssist(GenericHID hid) {
        HIDRumble.rumble(hid, new RumbleRequest(RumbleType.kLeftRumble, 0.5, 0.25));
    }

    private RumbleFeedback() {}
}
