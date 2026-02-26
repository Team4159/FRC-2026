package frc.robot.lib;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.JoeLookupTableConstants;
import frc.robot.Constants.JoeLookupTableConstants.ShotData;

public class JoeLookupTable {
    /** @param distance distance away from hub 
     * @return the corresponding ShotData object (time and angle) from linearly interpolating the best 2 joeLookupTable points
    */
    public static ShotData getShotData(Distance distance){
        Distance bestFitDistance = Inches.of(Double.MAX_VALUE);
        Distance secondBestFitDistance = Inches.of(Double.MAX_VALUE);

        //get keys from table (A Java Map)
        Set<Distance> keys = JoeLookupTableConstants.joeLookupTable.keySet();

        //loop through each key
        for(Distance currentDistance : keys){
            //find best fit and second best fit distances
            if(currentDistance.minus(distance).abs(Inches) < bestFitDistance.minus(distance).abs(Inches)){
                secondBestFitDistance = bestFitDistance;
                bestFitDistance = currentDistance;
            }
            else if(currentDistance.minus(distance).abs(Inches) < secondBestFitDistance.minus(distance).abs(Inches)){
                secondBestFitDistance = currentDistance;
            }
        }
        //get the ShotData objects from the 2 best fit points
        ShotData bestFitShotData = JoeLookupTableConstants.joeLookupTable.get(bestFitDistance);
        ShotData secondBestFitShotData = JoeLookupTableConstants.joeLookupTable.get(secondBestFitDistance);

        //convert the angles and times to doubles(for linear interpolation)
        double bestFitAngle = bestFitShotData.getAngleRadians();
        double secondBestFitAngle = secondBestFitShotData.getAngleRadians();
        double bestFitTime = bestFitShotData.getTimeSeconds();
        double secondBestFitTime = secondBestFitShotData.getTimeSeconds();

        //get the interpolation point
        double linearInterpolation = 
            distance.minus(bestFitDistance).abs(Inches) / 
            (bestFitDistance.minus(distance).abs(Inches) + secondBestFitDistance.minus(distance).abs(Inches));

        SmartDashboard.putNumber("linear interpolation", linearInterpolation);
        SmartDashboard.putNumber("bestFitDistance", bestFitDistance.in(Meters));

        //return a new ShotData object with the interpolated time and angle.
        return new ShotData(
            Radians.of(MathUtil.interpolate(bestFitAngle, secondBestFitAngle, linearInterpolation)), 
            Seconds.of(MathUtil.interpolate(bestFitTime, secondBestFitTime, linearInterpolation)));
    }
}
