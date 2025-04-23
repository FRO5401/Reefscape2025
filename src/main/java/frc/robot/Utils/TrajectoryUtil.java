package frc.robot.Utils;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

public class TrajectoryUtil {
    public static Trajectory generateTrajectory(Supplier<Pose2d> initalPose, Pose2d finalPose, List<Translation2d> midPoints){
        return TrajectoryGenerator.generateTrajectory(
                        initalPose.get(),
                        midPoints,
                        finalPose,
                        Constants.Trajectorys.config);
    }
}
