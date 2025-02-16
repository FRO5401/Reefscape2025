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
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

public class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
    /** Running on a real robot. */
        REAL,

    /** Running a physics simulator. */
        SIM,

    /** Replaying from a log file. */
        REPLAY
    }
    
    public static class ControlConstants{
        //  Controller Specifications
        public static final int XBOX_CONTROLLER_DRIVER = 0;
        public static final int XBOX_CONTROLLER_OPERATOR = 1;
        public static final double CONTROLLER_SENSITIVITY = 0.05;
    }

    public class ElevatorConstants{

        public static final int elevatorID = 14;
        public static final double EXTENDED_POSITION = 31;
        public static final double HALF_EXTENDED_POSITION = 15.5;
        public static final double KP = 3; // An error of 1 rotation results in 2.4 V output
        public static final double KI = 3; // no output for integrated error
        public static final double KD = .42; // A velocity of 1 rps results in 0.1 V output
        public static final double KA=.1;
        public static final double KV=.1;
        public static final double KG=2.4;

        /*  Constants for the Simulation Elevator */
        public class ElevatorSimConstants{
            public static final int kEncoderAChannel = 0;
            public static final int kEncoderBChannel = 1;          
          
            public static final double kElevatorkS = 0.0; // volts (V)
            public static final double kElevatorkG = 0.762; // volts (V)
            public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
          
            public static final double ElevatorGearing = 5.0;
            public static final double kElevatorDrumRadius = Units.inchesToMeters(1.125);
            public static final double kCarriageMass = Units.lbsToKilograms(12.5);
          
            public static final double kSetpointMeters = 0.75;
            // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
            public static final double kMinElevatorHeightMeters = Units.inchesToMeters(41.19);
            public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(97.85);
            public static final double setPointMeters = Units.inchesToMeters(85);
            
          
            // distance per pulse = (distance per revolution) / (pulses per revolution) = (Pi * D) / ppr
            public static final double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * kElevatorDrumRadius / 4096;
        }
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
        public static final double robotWidth = 28;
        public static final double robotLength = 32;

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
