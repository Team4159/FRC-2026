package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.JoeLookupTable;
import frc.lib.JoeLookupTable.LookupTablePoint;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PhysicsConstants;
import frc.robot.subsystems.shooter.ShooterConstants.AutoShootStatus;

public class ShotCalculator {

    public enum ShotCalculatorStatus {
        SUCCESS,
        OUT_OF_RANGE,
    }

    public record ShotCalculatorResult(
        ShotCalculatorStatus status,
        AngularVelocity angularVelocity,
        Angle pitch,
        Angle yaw
    ) {
        public ShotCalculatorResult(ShotCalculatorStatus status) {
            this(status, RPM.of(0.0), Radians.of(0.0), Radians.of(0.0));
        }
    }

    private static final double HEIGHT = FieldConstants.HUB_Z - Units.inchesToMeters(20);

    private ShotCalculator() {}

    private ShotCalculatorResult calculate(Translation2d target, Translation2d translation, ChassisSpeeds speeds) {
        // calculate desired pitch for hood angle
        AngularVelocity angularVelocity = RPM.of(0.0);
        double efficiency = 1.0;
        Angle pitch = getPitch(target, translation, angularVelocity, efficiency);
        Translation2d adjustedTranslation = translation;
        for (int i = 0; i < 2; i++) {
            // get desired angular velocity and efficiency from lookup table
            LookupTablePoint lookupTablePoint = JoeLookupTable.getLookupTablePoint(Meters.of(getDistanceFromTarget()));
            angularVelocity = lookupTablePoint.angularVelocity();
            efficiency = lookupTablePoint.efficiency();
            // calculate TOF(used for calculating adjusted robot pose)
            double timeOfFlight = getTimeOfFlight(pitch, getAngularVelocity(angularVelocity));
            // add the distance traveled during TOF to current robot pose to get the
            // adjusted robot pose
            // this will be used for shooting while moving adjustment
            adjustedTranslation = translation.plus(
                new Translation2d(speeds.vxMetersPerSecond * timeOfFlight, speeds.vyMetersPerSecond * timeOfFlight)
            );

            // recalculate desired hood angle with new adjustedPose (converges)
            pitch = getPitch(target, translation, angularVelocity, efficiency);
        }

        // get desired angular velocity and efficiency from lookup table
        LookupTablePoint lookupTablePoint = JoeLookupTable.getLookupTablePoint(Meters.of(getDistanceFromTarget()));
        angularVelocity = lookupTablePoint.angularVelocity();
        efficiency = lookupTablePoint.efficiency();

        // check if in range, return if out of range
        if (getDistanceFromTarget() > JoeLookupTableConstants.MAX_DISTANCE.in(Meters)) {
            return new ShotCalculatorResult(ShotCalculatorStatus.OUT_OF_RANGE);
        }

        // calculate robot theta based on adjusted robot pose
        // this allows for shooting while moving
        Angle yaw = target.minus(adjustedTranslation).getAngle().getMeasure();

        return new ShotCalculatorResult(ShotCalculatorStatus.SUCCESS, angularVelocity, pitch, yaw);
    }

    private Angle getPitch(
        Translation2d target,
        Translation2d translation,
        AngularVelocity angularVelocity,
        double efficiency
    ) {
        double distance = translation.getDistance(target);
        double exitVelocity = calculateExitVelocity(angularVelocity, efficiency).baseUnitMagnitude();
        double desiredPitch = Math.atan(
            (Math.pow(exitVelocity, 2) +
                Math.sqrt(
                    Math.pow(exitVelocity, 4) -
                        Math.pow(PhysicsConstants.GRAVITY * distance, 2) -
                        2 * PhysicsConstants.GRAVITY * HEIGHT * Math.pow(exitVelocity, 2)
                )) /
                (PhysicsConstants.GRAVITY * distance)
        );

        if (Double.isNaN(desiredPitch)) {
            // equation can only return angles from 45-90 deg (in radians of course),
            // anything lower than that will be NaN
            // the minimum possible hood angle on the physical shooter is 45, so no
            // additional calculation is needed, just set to 45
            desiredPitch = Units.degreesToRadians(45);
            autoShootStatus = AutoShootStatus.OUT_OF_RANGE;
        }
        desiredPitch = Math.min(desiredPitch, HoodConstants.MAX_PITCH.in(Radians));
        SmartDashboard.putNumber("autoaim desired pitch", Units.radiansToDegrees(desiredPitch));
        return Radians.of(desiredPitch);
    }

    private LinearVelocity calculateExitVelocity(AngularVelocity angularVelocity, double efficiency) {
        double angularVelocityDouble = angularVelocity.in(RadiansPerSecond) * FlywheelConstants.ROTOR_TO_WHEEL_RATIO;

        double wheelTangentialSpeed = angularVelocityDouble * FlywheelConstants.WHEEL_RADIUS.in(Meters);
        double rollerTangentialSpeed = angularVelocityDouble * FlywheelConstants.ROLLER_RADIUS.in(Meters);

        return MetersPerSecond.of((efficiency * (wheelTangentialSpeed + rollerTangentialSpeed)) / 2);
    }
}
