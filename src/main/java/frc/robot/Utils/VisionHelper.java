package frc.robot.Utils;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.VisionConstants;

public class VisionHelper {
    public static Pose3d[] getTagPoses(PhotonPipelineResult results){
        if(!results.getTargets().isEmpty()){
            Pose3d[] poses = new Pose3d[results.targets.size()];
            var targets = results.getTargets();
            for(int i=0; i<targets.size(); i++){
                poses[i] = VisionConstants.aprilTagLayout.getTagPose(results.getTargets().get(i).fiducialId).get();
            }
            return poses;
        }
        else return new Pose3d[]{};
        
    }
}
