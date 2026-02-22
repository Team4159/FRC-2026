package frc.lib;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HubTracker {

    public static boolean isHubActive(Alliance alliance) {
        Optional<Alliance> activeHub = getActiveHub();
        if (!activeHub.isEmpty()) {
            return alliance.equals(activeHub.get());
        }
        return false;
    }

    public static Optional<Alliance> getActiveHub() {
        double matchTime = DriverStation.getMatchTime();
        Optional<Alliance> autoWinnerOptional = getAutoWinner();
        if (matchTime > 130 || matchTime <= 30 || autoWinnerOptional.isEmpty()) {
            return Optional.empty();
        }

        Alliance autoWinner = autoWinnerOptional.get();
        Alliance autoLoser = autoWinner.equals(Alliance.Blue) ? Alliance.Red : Alliance.Blue;
        if (matchTime > 105) {
            return Optional.of(autoLoser);
        } else if (matchTime > 80) {
            return Optional.of(autoWinner);
        } else if (matchTime > 55) {
            return Optional.of(autoLoser);
        } else if (matchTime > 30) {
            return Optional.of(autoWinner);
        }

        return Optional.empty();
    }

    public static Optional<Double> getTimeUntilNextActiveHub() {
        double matchTime = DriverStation.getMatchTime();

        if (matchTime > 140 || matchTime == -1) {
            return Optional.empty();
        }

        if (matchTime > 105) {
            return Optional.of(matchTime - 105);
        } else if (matchTime > 80) {
            return Optional.of(matchTime - 80);
        } else if (matchTime > 55) {
            return Optional.of(matchTime - 55);
        } else if (matchTime > 30) {
            return Optional.of(matchTime - 30);
        }

        return Optional.empty();
    }

    public static Optional<Alliance> getAutoWinner() {
        String gameData = DriverStation.getGameSpecificMessage();
        return switch (gameData.length() > 0 ? gameData.charAt(0) : ' ') {
            case 'B' -> Optional.of(Alliance.Blue);
            case 'R' -> Optional.of(Alliance.Red);
            default -> Optional.empty();
        };
    }

    private HubTracker() {
    }
}
