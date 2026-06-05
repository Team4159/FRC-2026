package frc.lib;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class PoseTrajectory extends Trajectory{
    public PoseTrajectory(){
        super();
    }

    /** @param poses an ArrayList of Pose2d objects that make up the trajectory
     * literally the entire reason this exists is to input poses into a trajectory that can then be displayed on a field2d 😭😭😭
     */
    public PoseTrajectory(ArrayList<Pose2d> poses){
        super();
        //get the array of "State" objects (Pose with additional trajectory data for the goofy WPILIB path follower tool)
        var states = getStates();
        //loop through all the poses
        for(int i = 0; i < poses.size(); i++){
            //we dont care about the other data so just set it to I but set the pose so that the trajectory will be displayed
            states.add(new State(i, i, i, poses.get(i), i));
        }
    }
}
