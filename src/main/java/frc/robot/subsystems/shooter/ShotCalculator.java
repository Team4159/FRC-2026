package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.PhysicsConstants;
import frc.robot.subsystems.shooter.JoeLookupTable.LookupTablePoint;

public class ShotCalculator {

    public enum ShotCalculatorStatus {
        SUCCESS,
        OUT_OF_RANGE,
    }

    public record ShotCalculatorResult(
        ShotCalculatorStatus status,
        LinearVelocity tangentialVelocity,
        Angle pitch,
        Angle yaw
    ) {
        public ShotCalculatorResult(ShotCalculatorStatus status) {
            this(status, MetersPerSecond.of(0.0), Radians.of(0.0), Radians.of(0.0));
        }
    }

    private ShotCalculator() {}

    public static LinearVelocity angularVelocityToTangentialVelocity(AngularVelocity angularVelocity) {
        return MetersPerSecond.of(
            angularVelocity.baseUnitMagnitude() *
                ((FlywheelConstants.ROLLER_RADIUS.baseUnitMagnitude() +
                    HoodConstants.ROLLER_RADIUS.baseUnitMagnitude()) /
                    2.0)
        );
    }

    public static AngularVelocity tangentialVelocityToAngularVelocity(LinearVelocity tangentialVelocity) {
        return RadiansPerSecond.of(
            tangentialVelocity.baseUnitMagnitude() /
                ((FlywheelConstants.ROLLER_RADIUS.baseUnitMagnitude() +
                    HoodConstants.ROLLER_RADIUS.baseUnitMagnitude()) /
                    2.0)
        );
    }

    public static boolean inRange(double distance) {
        return distance <= JoeLookupTableConstants.MAX_DISTANCE.baseUnitMagnitude();
    }

    public static boolean inRange(Translation2d t1, Translation2d t2) {
        return inRange(t1.getDistance(t2));
    }

    public static ShotCalculatorResult calculate(
        Translation2d target,
        Translation2d translation,
        ChassisSpeeds fieldSpeeds
    ) {
        double distanceToTarget = translation.getDistance(target);
        // check if in range, return if out of range
        if (!inRange(distanceToTarget)) {
            return new ShotCalculatorResult(ShotCalculatorStatus.OUT_OF_RANGE);
        }

        // calculate desired pitch for hood angle
        LookupTablePoint lookupTablePoint = JoeLookupTable.getLookupTablePoint(Meters.of(distanceToTarget));
        double tangentialVelocity = angularVelocityToTangentialVelocity(
            lookupTablePoint.angularVelocity()
        ).baseUnitMagnitude();
        double efficiency = lookupTablePoint.efficiency();
        double exitVelocity = calculateExitVelocity(tangentialVelocity, efficiency);

        Translation2d adjustedTranslation = translation;
        double pitch = calculatePitch(distanceToTarget, tangentialVelocity, efficiency);
        for (int i = 0; i < 5; i++) {
            // calculate TOF(used for calculating adjusted robot pose)
            double timeOfFlight = calculateTimeOfFlight(pitch, exitVelocity);
            // add the distance traveled during TOF to current robot pose to get the
            // adjusted robot pose
            // this will be used for shooting while moving adjustment
            adjustedTranslation = translation.plus(
                new Translation2d(
                    fieldSpeeds.vxMetersPerSecond * timeOfFlight,
                    fieldSpeeds.vyMetersPerSecond * timeOfFlight
                )
            );
            // recalculate desired hood angle with new adjustedPose (converges)
            pitch = calculatePitch(adjustedTranslation.getDistance(target), tangentialVelocity, efficiency);
        }

        // calculate robot theta based on adjusted robot pose
        // this allows for shooting while moving
        double yaw = target.minus(adjustedTranslation).getAngle().getRadians();

        return new ShotCalculatorResult(
            ShotCalculatorStatus.SUCCESS,
            MetersPerSecond.of(tangentialVelocity),
            Radians.of(pitch),
            Radians.of(yaw)
        );
    }

    private static double calculatePitch(double distance, double tangentialVelocity, double efficiency) {
        double exitVelocity = calculateExitVelocity(tangentialVelocity, efficiency);
        double desiredPitch = Math.atan(
            (Math.pow(exitVelocity, 2) +
                Math.sqrt(
                    Math.pow(exitVelocity, 4) -
                        Math.pow(PhysicsConstants.GRAVITY * distance, 2) -
                        2 * PhysicsConstants.GRAVITY * JoeLookupTableConstants.TARGET_HEIGHT * Math.pow(exitVelocity, 2)
                )) /
                (PhysicsConstants.GRAVITY * distance)
        );

        if (Double.isNaN(desiredPitch)) {
            // equation can only return angles from 45-90 deg (in radians of course),
            // anything lower than that will be NaN
            // the minimum possible hood angle on the physical shooter is 45, so no
            // additional calculation is needed, just set to 45
            desiredPitch = Units.degreesToRadians(45);
        }
        desiredPitch = Math.min(desiredPitch, HoodConstants.MAX_PITCH.in(Radians));
        return desiredPitch;
    }

    private static double calculateExitVelocity(double tangentialVelocity, double efficiency) {
        double angularVelocity = tangentialVelocityToAngularVelocity(
            MetersPerSecond.of(tangentialVelocity)
        ).baseUnitMagnitude();
        double wheelTangentialSpeed = angularVelocity * FlywheelConstants.ROLLER_RADIUS.baseUnitMagnitude();
        double rollerTangentialSpeed = angularVelocity * HoodConstants.ROLLER_RADIUS.baseUnitMagnitude();
        return ((wheelTangentialSpeed + rollerTangentialSpeed) / 2.0) * efficiency;
    }

    private static double calculateTimeOfFlight(double pitch, double exitVelocity) {
        // initial y component of launch velocity
        double vy = exitVelocity * Math.sin(pitch);
        // the calculation is based on delta y = vy * TOF - (1/2)g * TOF^2 (where g is a
        // positive constant)
        // the delta y for TOF would be the height
        // the equation then becomes 0 = -(1/2)g * TOF^2 + vy * TOF - height -> 0 =
        // (1/2)g * TOF^2 - vy * TOF + height
        // then use quadratic formula and always add the radical to get the 2nd time the
        // fuel is at the target height (so that it is on the way down)
        double radical = Math.sqrt(
            Math.pow(vy, 2) - 2 * PhysicsConstants.GRAVITY * JoeLookupTableConstants.TARGET_HEIGHT
        );
        if (Double.isNaN(radical)) {
            return 0.0;
        }
        double numerator = vy + radical;
        double time = numerator / PhysicsConstants.GRAVITY;
        return time;
    }
}
