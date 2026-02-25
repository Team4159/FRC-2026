package frc.robot.lib;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class PoseTrajectory extends Trajectory{
    public PoseTrajectory(){
        super();
    }

    public PoseTrajectory(ArrayList<Pose2d> poses){
        super();
        var states = getStates();
        for(int i = 0; i < poses.size(); i++){
            states.add(new State(i, i, i, poses.get(i), i));
        }
    }
}
