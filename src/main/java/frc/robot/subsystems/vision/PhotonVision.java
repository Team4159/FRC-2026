package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision extends SubsystemBase {

    private final Drivetrain drivetrain;

    private final Field2d testField = new Field2d();

    public PhotonVision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        testField.setRobotPose(drivetrain.getState().Pose);
        addCameraToTestField("leftCamera", PhotonVisionConstants.LEFT_SHOOTER_CAMERA_TRANSFORM);
        addCameraToTestField("rightCamera", PhotonVisionConstants.RIGHT_SHOOTER_CAMERA_TRANSFORM);
        SmartDashboard.putData("Pose Testing", testField);
    }

    @Override
    public void periodic() {
        estimate(PhotonVisionConstants.LEFT_SHOOTER_CAMERA, PhotonVisionConstants.LEFT_SHOOTER_POSE_ESTIMATOR);
        estimate(PhotonVisionConstants.RIGHT_SHOOTER_CAMERA, PhotonVisionConstants.RIGHT_SHOOTER_POSE_ESTIMATOR);
    }

    private void estimate(PhotonCamera camera, PhotonPoseEstimator estimator) {
        //loops through all unread camera results
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            //get pose estimate
            Optional<EstimatedRobotPose> estimate = estimator.estimateCoprocMultiTagPose(result);
            //multitag no longer defaults to single tag when no others are available so we have this
            if (!estimate.isPresent()) {
                estimate = estimator.estimateLowestAmbiguityPose(result);
            }
            //check if estimate exists
            if (!estimate.isPresent()) {
                continue;
            }
            if (result.getBestTarget().getPoseAmbiguity() > PhotonVisionConstants.POSE_AMBIGUITY_THRESHOLD) {
                continue;
            }
            //set standard deviation
            drivetrain.setVisionMeasurementStdDevs(
                calculateEstimationStdDevs(
                    estimate,
                    result.targets,
                    estimator,
                    PhotonVisionConstants.SINGLE_TAG_STANDARD_DEVIATION,
                    PhotonVisionConstants.MULTI_TAG_STANDARD_DEVIATION
                )
            );
            //send the pose estimate to the pose estimator
            drivetrain.addVisionMeasurement(estimate.get().estimatedPose.toPose2d(), estimate.get().timestampSeconds);
        }
    }

    // private Vector<N3> calculateEstimationStdDevs(
    //     Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    //     //range should be form 0(no tag) to 1(full coverage) (limelight standard)
    //     //photonvision area is scaled from 0-100 so need to convert
    //     double area = 0;
    //     //calculate area of all targets
    //     if(estimatedPose.isPresent()){
    //         for(var tag : targets){
    //             //convert the scaling
    //             area += tag.area/100;
    //         }
    //         System.out.println("stddev position: " + (1 - area * 0.3));
    //         //TODO: tune, currently this is just the limelight one(why are sds on limelight negative lol)
    //         //return VecBuilder.fill(1 - area * 0.3, 1 - area * 0.3, 1-area * 0.1);
    //         return VecBuilder.fill(-1, -1, -1);
    //     }else{
    //         System.out.println("cooked :(");
    //     }
    //     //if the estimated pose does not exist just return extremely high stddevs
    //     return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    // }

    private Matrix<N3, N1> calculateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose,
        List<PhotonTrackedTarget> targets,
        PhotonPoseEstimator photonEstimator,
        Matrix<N3, N1> singleTagStdDevs,
        Matrix<N3, N1> multiTagStdDevs
    ) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            return singleTagStdDevs;
        }
        // Pose present. Start running Heuristic
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0.0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            numTags++;
            avgDist += tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            // No tags visible. Default to single-tag std devs
            return singleTagStdDevs;
        }
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = multiTagStdDevs;
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist) / 60);
        }
        return estStdDevs;
    }

    private void addCameraToTestField(String name, Transform3d transform) {
        testField
            .getObject(name)
            .setPose(
                drivetrain
                    .getState()
                    .Pose.plus(
                        new Transform2d(transform.getX(), transform.getY(), transform.getRotation().toRotation2d())
                    )
            );
    }
}
