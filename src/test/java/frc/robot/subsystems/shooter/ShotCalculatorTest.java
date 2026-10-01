package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.shooter.ShotCalculator.ShotCalculatorStatus;
import org.junit.jupiter.api.Test;

class ShotCalculatorTest {

    @Test
    void convertsAngularAndTangentialVelocityUsingMeanRollerRadius() {
        var angular = RPM.of(1200);
        var tangential = ShotCalculator.angularVelocityToTangentialVelocity(angular);
        var roundTrip = ShotCalculator.tangentialVelocityToAngularVelocity(tangential);

        assertEquals(angular.in(RPM), roundTrip.in(RPM), 1e-9);
    }

    @Test
    void acceptsDistancesThroughConfiguredMaximum() {
        assertTrue(ShotCalculator.inRange(0));
        assertTrue(ShotCalculator.inRange(JoeLookupTableConstants.MAX_DISTANCE.baseUnitMagnitude()));
        assertFalse(ShotCalculator.inRange(JoeLookupTableConstants.MAX_DISTANCE.baseUnitMagnitude() + 0.001));
    }

    @Test
    void returnsOutOfRangeResultBeyondLookupRange() {
        var result = ShotCalculator.calculate(
            new Translation2d(0, 0),
            new Translation2d(JoeLookupTableConstants.MAX_DISTANCE.in(Meters) + 0.1, 0),
            new ChassisSpeeds()
        );

        assertEquals(ShotCalculatorStatus.OUT_OF_RANGE, result.status());
        assertEquals(0.0, result.tangentialVelocity().in(MetersPerSecond));
    }

    @Test
    void computesFiniteStationaryShotSolutionInRange() {
        var result = ShotCalculator.calculate(
            new Translation2d(0, 0),
            new Translation2d(2, 0),
            new ChassisSpeeds()
        );

        assertEquals(ShotCalculatorStatus.SUCCESS, result.status());
        assertTrue(Double.isFinite(result.tangentialVelocity().in(MetersPerSecond)));
        assertTrue(Double.isFinite(result.pitch().baseUnitMagnitude()));
        assertEquals(Math.PI, Math.abs(result.yaw().baseUnitMagnitude()), 1e-9);
    }

    @Test
    void accountsForRobotTranslationWhenCalculatingYaw() {
        var stationary = ShotCalculator.calculate(
            new Translation2d(0, 0), new Translation2d(2, 0), new ChassisSpeeds()
        );
        var moving = ShotCalculator.calculate(
            new Translation2d(0, 0), new Translation2d(2, 0), new ChassisSpeeds(0, 1, 0)
        );

        assertNotEquals(stationary.yaw().baseUnitMagnitude(), moving.yaw().baseUnitMagnitude(), 1e-6);
    }
}
