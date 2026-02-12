package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVision extends SubsystemBase{
    private Drivetrain drivetrain;
    //IDK what the difference between welded and andymark is
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    //TODO: add other cameras later once we know where they are on the robot
    private PhotonCamera leftShooterCam, rightShooterCam;
    private PhotonPoseEstimator leftShooterEstimator, rightShooterEstimator;

    public PhotonVision(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        //cameras
        leftShooterCam = new PhotonCamera("leftShooter");
        rightShooterCam = new PhotonCamera("rightShooter");
        //estimators
        leftShooterEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, Constants.PhotonVisionConstants.leftShooterCamTransform);
        rightShooterEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, Constants.PhotonVisionConstants.rightShooterCamTransform);
    }


    @Override
    public void periodic(){
        //left camera
        Optional<EstimatedRobotPose> leftShooterEstimate = Optional.empty();
        //loops through all unread camera results
        for(PhotonPipelineResult leftShooterCamResult : leftShooterCam.getAllUnreadResults()){
            //get pose estimate
            leftShooterEstimate = leftShooterEstimator.estimateCoprocMultiTagPose(leftShooterCamResult);
            //multitag no longer defaults to single tag when no others are available so we have this
            if(!leftShooterEstimate.isPresent()){
                leftShooterEstimate = rightShooterEstimator.estimateLowestAmbiguityPose(leftShooterCamResult);
            }
            //check if estimate exists
            if(leftShooterEstimate.isPresent()){
                //set standard deviation
                drivetrain.setVisionMeasurementStdDevs(calculateEstimationStdDevs(leftShooterEstimate, leftShooterCamResult.targets));
                //send the pose estimate to the pose estimator
                drivetrain.addVisionMeasurement(leftShooterEstimate.get().estimatedPose.toPose2d(), leftShooterEstimate.get().timestampSeconds);
            }

        }

        //right camera
        Optional<EstimatedRobotPose> rightShooterEstimate = Optional.empty();
        //loops through all unread camera results
        for(PhotonPipelineResult rightShooterCamResult : rightShooterCam.getAllUnreadResults()){
            //get pose estimate
            rightShooterEstimate = rightShooterEstimator.estimateCoprocMultiTagPose(rightShooterCamResult);
            //multitag no longer defaults to single tag when no others are available so we have this
            if(!rightShooterEstimate.isPresent()){
                rightShooterEstimate = rightShooterEstimator.estimateLowestAmbiguityPose(rightShooterCamResult);
            }
            //check if estimate exists
            if(rightShooterEstimate.isPresent()){
                //set standard deviation
                drivetrain.setVisionMeasurementStdDevs(calculateEstimationStdDevs(rightShooterEstimate, rightShooterCamResult.targets));
                //send the pose estimate to the pose estimator
                drivetrain.addVisionMeasurement(rightShooterEstimate.get().estimatedPose.toPose2d(), rightShooterEstimate.get().timestampSeconds);
            }
        }
    }

    //default photonvision stddev calculator
    // private Vector<N3> calculateEstimationStdDevs(
    //         Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator photonEstimator) {
    //         Vector<N3> stddevs;
    //     if (estimatedPose.isEmpty()) {
    //         // No pose input. Default to single-tag std devs
    //         stddevs = kSingleTagStdDevs;

    //     } else {
    //         // Pose present. Start running Heuristic
    //         var estStdDevs = kSingleTagStdDevs;
    //         int numTags = 0;
    //         double avgDist = 0;

    //         // Precalculation - see how many tags we found, and calculate an average-distance metric
    //         for (var tgt : targets) {
    //             var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
    //             if (tagPose.isEmpty()) continue;
    //             numTags++;
    //             avgDist +=
    //                     tagPose
    //                             .get()
    //                             .toPose2d()
    //                             .getTranslation()
    //                             .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
    //         }

    //         if (numTags == 0) {
    //             // No tags visible. Default to single-tag std devs
    //             stddevs = kSingleTagStdDevs;
    //         } else {
    //             // One or more tags visible, run the full heuristic.
    //             avgDist /= numTags;
    //             // Decrease std devs if multiple targets are visible
    //             if (numTags > 1) estStdDevs = kMultiTagStdDevs;
    //             // Increase std devs based on (average) distance
    //             if (numTags == 1 && avgDist > 4)
    //                 estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    //             else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    //             stddevs = estStdDevs;
    //         }
    //     }
    //     return stddevs;
    // }

    private Vector<N3> calculateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        //range should be form 0(no tag) to 1(full coverage) (limelight standard)
        //photonvision area is scaled from 0-100 so need to convert
        double area = 0;
        //calculate area of all targets
        if(estimatedPose.isPresent()){
            for(var tag : targets){
                //convert the scaling
                area += tag.area/100; 
            }
            System.out.println("stddev position: " + (1 - area * 0.3));
            //TODO: tune, currently this is just the limelight one(why are sds on limelight negative lol)
            //return VecBuilder.fill(1 - area * 0.3, 1 - area * 0.3, 1-area * 0.1);
            return VecBuilder.fill(-1, -1, -1);
        }else{
            System.out.println("cooked :(");
        }
        //if the estimated pose does not exist just return extremely high stddevs
        return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
}
