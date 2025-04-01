package frc.robot.Utils;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;

public class VisionHelper {
    public static Pose2d[] getTagPoses(PhotonPipelineResult results){
        if(results.targets.size() != 0){
        Pose2d[] poses = new Pose2d[results.targets.size()];
        var targets = results.getTargets();
        for(int i=0; i<targets.size(); i++){
            poses[i] = VisionConstants.aprilTagLayout.getTagPose(results.getTargets().get(i).fiducialId).get().toPose2d();
        }
        return poses;
        }
        else return new Pose2d[]{};
    }
}
