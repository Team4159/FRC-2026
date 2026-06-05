package frc.lib;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.JoeLookupTableConstants;
import frc.robot.Constants.JoeLookupTableConstants.LookupTablePoint;
import java.util.Set;

public class JoeLookupTable {

    /** @param distance distance away from hub
     * @return the corresponding ShotData object (desired motor angular velocity and efficiency) from linearly interpolating the best 2 joeLookupTable points
     */
    public static LookupTablePoint getLookupTablePoint(Distance distance) {
        //set the best and second best fit distances to the max double value (because it will never be that close)
        //cannot use the 1st and second data points because if second best fit distance is accidentally set to the best fit distance the calculation wont work
        Distance bestFitDistance = Inches.of(Double.MAX_VALUE);
        Distance secondBestFitDistance = Inches.of(Double.MAX_VALUE);

        //get keys from the lookup table (A Java Map)
        Set<Distance> keys = JoeLookupTableConstants.joeLookupTable.keySet();

        //loop through each key
        for (Distance currentDistance : keys) {
            //find best fit and second best fit distances
            if (currentDistance.minus(distance).abs(Inches) < bestFitDistance.minus(distance).abs(Inches)) {
                //if a new best fit distance is found, set the current best fit to the second best fit and then set the best fit to the new best fit
                secondBestFitDistance = bestFitDistance;
                bestFitDistance = currentDistance;
            } else if (
                currentDistance.minus(distance).abs(Inches) < secondBestFitDistance.minus(distance).abs(Inches)
            ) {
                //if a new second best fit is found, set the second best fit to the new second best fit
                secondBestFitDistance = currentDistance;
            }
        }
        //get avs and efficiencies as doubles from 2 closest points
        //AV means angular velocity btw
        LookupTablePoint bestFitPoint = JoeLookupTableConstants.joeLookupTable.get(bestFitDistance);
        LookupTablePoint secondBestFitPoint = JoeLookupTableConstants.joeLookupTable.get(secondBestFitDistance);
        double bestFitAV = bestFitPoint.angularVelocity().in(RPM);
        double secondBestFitAV = secondBestFitPoint.angularVelocity().in(RPM);
        double bestFitEfficiency = bestFitPoint.efficiency();
        double secondBestFitEfficiency = secondBestFitPoint.efficiency();

        //get the interpolation point
        double linearInterpolation =
            distance.minus(bestFitDistance).abs(Inches) /
            (bestFitDistance.minus(distance).abs(Inches) + secondBestFitDistance.minus(distance).abs(Inches));

        SmartDashboard.putNumber("linear interpolation", linearInterpolation);
        SmartDashboard.putNumber("bestFitDistance", bestFitDistance.in(Meters));

        //return a new ShotData object with the interpolated angular velocity and efficiency.
        return new LookupTablePoint(
            RPM.of(MathUtil.interpolate(bestFitAV, secondBestFitAV, linearInterpolation)),
            MathUtil.interpolate(bestFitEfficiency, secondBestFitEfficiency, linearInterpolation)
        );
    }
}
