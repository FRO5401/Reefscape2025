package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public class Constants {
    public static class ControlConstants{
        //  Controller Specifications
        public static final int XBOX_CONTROLLER_DRIVER = 0;
        public static final int XBOX_CONTROLLER_OPERATOR = 1;
        public static final double CONTROLLER_SENSITIVITY = 0.05;
    }
    public class ElevatorConstants{
        public static final int elevatorID = 14;
        public static final double testPosition = 5;
    }
    public class AutoConstants{

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = .175;
        public static final double kPYController = 2.5;
        public static final double kPThetaController = .25;
    
        /* Constraint for kpx motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);


        public static final ProfiledPIDController thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

            
        
    };

    public static final class Swerve{
        public static final double trackWidth = Units.inchesToMeters(22.75); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(26.75); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = 4 *Math.PI;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
    }

    public static final class Trajectorys {
        //config to ensure the robot doesnt try to move through itself
        public static final  TrajectoryConfig config = new TrajectoryConfig(
                    TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
                    //TODO: Fix this to be the actual constant. 
                    3.23)
                .setKinematics(Constants.Swerve.swerveKinematics);


        public final static Trajectory sCurveTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0,0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1,1), new Translation2d(-1, 3)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 5, new Rotation2d(Units.degreesToRadians(90))),
                config);

        public final static Trajectory rSCurveTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0,5, new Rotation2d(Units.degreesToRadians(90))),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-1,3), new Translation2d(1, 1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 0, new Rotation2d(0)),
                config);
    };

}
