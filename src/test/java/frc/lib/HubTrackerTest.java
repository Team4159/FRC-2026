package frc.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class HubTrackerTest {

    @Test
    void decodesAutonomousWinnerFromGameData() {
        assertEquals(Optional.of(Alliance.Blue), HubTracker.getAutoWinner("B"));
        assertEquals(Optional.of(Alliance.Red), HubTracker.getAutoWinner("R123"));
    }

    @Test
    void handlesMissingOrInvalidGameDataWithoutThrowing() {
        assertEquals(Optional.empty(), HubTracker.getAutoWinner(""));
        assertEquals(Optional.empty(), HubTracker.getAutoWinner(null));
        assertEquals(Optional.empty(), HubTracker.getAutoWinner("X"));
    }

    @Test
    void alternatesActiveHubAcrossMatchWindows() {
        assertEquals(Optional.empty(), HubTracker.getActiveHub(131, Optional.of(Alliance.Blue)));
        assertEquals(Optional.of(Alliance.Red), HubTracker.getActiveHub(120, Optional.of(Alliance.Blue)));
        assertEquals(Optional.of(Alliance.Blue), HubTracker.getActiveHub(100, Optional.of(Alliance.Blue)));
        assertEquals(Optional.of(Alliance.Red), HubTracker.getActiveHub(60, Optional.of(Alliance.Blue)));
        assertEquals(Optional.of(Alliance.Blue), HubTracker.getActiveHub(40, Optional.of(Alliance.Blue)));
        assertEquals(Optional.empty(), HubTracker.getActiveHub(30, Optional.of(Alliance.Blue)));
    }

    @Test
    void reportsTheNextHubChangeOnlyWhenWinnerIsKnown() {
        assertEquals(Optional.of(15.0), HubTracker.getTimeUntilNextActiveHub(120, true));
        assertEquals(Optional.of(20.0), HubTracker.getTimeUntilNextActiveHub(100, true));
        assertEquals(Optional.of(25.0), HubTracker.getTimeUntilNextActiveHub(80, true));
        assertEquals(Optional.empty(), HubTracker.getTimeUntilNextActiveHub(30, true));
        assertEquals(Optional.empty(), HubTracker.getTimeUntilNextActiveHub(120, false));
    }
}
