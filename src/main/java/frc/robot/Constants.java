package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import frc.robot.generated.TunerConstants;

public class Constants {
    public static class ControlConstants{
        //  Controller Specifications
        public static final int XBOX_CONTROLLER_DRIVER = 0;
        public static final int XBOX_CONTROLLER_OPERATOR = 1;
        public static final double CONTROLLER_SENSITIVITY = 0.05;
        
    }

    public static final class VisionConstants{
        public static final int[] blueReefTagIDs = {17, 18, 19, 20, 21, 22};
        public static final int[] redReefTagIDs = {6, 7, 8, 9, 10, 11};

        public static final int[] redStationTagIDS = {1,2};
        public static final int[] blueStationTagIDS = {12,13};

    
            public static AprilTagFieldLayout aprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

            public static final double REEF_DISTANCE = Units.feetToMeters(1.8);
            public static final double TELEOP_REEF_DISTANCE = Units.feetToMeters(1.6);
            public static final double ALGEA_DISTANCE = Units.feetToMeters(.8);
            public static final double BARGE_DISTANCE = Units.feetToMeters(4.1);


            public static final double AUTO_DISTANCE = Units.feetToMeters(3);

            public static final Transform3d ROBOT_TO_FRONT_CAM = new Transform3d(new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(11.5), Units.inchesToMeters(5)), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
            public static final Transform3d ROBOT_TO_RIGHT_CAM = new Transform3d(new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(-11.5), Units.inchesToMeters(5)), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    }

    public class ElevatorConstants{ 
        public static final int elevatorID = 13;
        public static final double SPEED_MODIFIER = 166;
        public static final double BARGE = 150-5 ;
        public static final double L4 = 140-9;//-6
        public static final double L3 = 82-5;//-3
        

        public static final double L2 = 53.91748046875-6;
        //TODO Fix this pls
        public static final double STATION = 52-2;//-7 at competition field
        public static final double PROCESSOR = 5;
        public static final double FLOOR = 1;

        public static final double KP = 4; // An error of 1 rotation results in 2.4 V output
        public static final double KI = 3; // no output for integrated error
        public static final double KD = .4; // A velocity of 1 rps results in 0.1 V output
        public static final double KA=.1;
        public static final double KV=.1;
        public static final double KG=2.4;
    }

    public class InfeedConstants{

        public static final int BEAM_BREAK_ID = 0;

        public class IntakeConstants{
            /*  ID of Spark Maxs */
            //Intake wheel motors
            public static final int INTAKE_MOTOR_LEFT = 20;
            public static final int INTAKE_MOTOR_RIGHT=18;
            //Rotation Motors
            public static final int ROTATE_MOTOR_LEFT=17; 
            public static final int ROTATE_MOTOR_RIGHT=15;

            /*  PIDF Values */
            public static final double ROTATE_KP = .1;
            public static final double ROTATE_kI = 0;
            public static final double ROTATE_kD = 0;
            public static final double ROTATE_KF = 10;

            /*  Position for each game piece */
            //Closes in to suck in coral
            public static final double HOLD_CORAL = 32;
            //Sides parallel to hold algea
            public static final double HOLD_ALGEA = 6;
            //Place coral 62.49993896484375
            public static final double TELEOP_REPEL_ALGEA = -0.58; //-0.7
            public static final double TELEOP_REPEL_CORAL = -.8; //-0.7
            public static final double AUTO_REPEL_ALGEA = -.58;
            public static final double INTAKE_SPEED = 1;
            
        }

        public class PivotConstants{
            // ID of Neo 1650
            public static final int PIVOT_ID=14;

            /*  PID values */
            public static final double PIVOT_KP=1;
            public static final double PIVOT_KI=0;
            public static final double PIVOT_KD=0;

            /*  Positon Values */
            //90 degree angle on the intake
            public static final double STRAIGHTOUT = -31*2;
            //public static final double PROCESSOR = 53; //Will replace straight out most likely
            //aimed up to shoot into the barge
            public static final double BARGE = -16;//-13*2
            //Scoring on top of barge
            public static final double L4 = -82*2;
            //takes algea out and possibly place on L2 and L3
            public static final double PLACE_CORAL = -62.49993896484375*2;
            public static final double CLEAR_ALGEA =-40.49993896484375*2;
            public static final double FLOOR_PICKUP =-50*2;

            //Gets Coral from station
            public static final double STATION = -25*2;
        }


    }
    public class AutoConstants{

        public static final double kMaxSpeedMetersPerSecond = 3.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1.687;
        public static final double kPYController = 1.687;
    
        public static final double kPThetaController = 1.687;
        public static final double kDThetaController = kPThetaController/2;
    
        /* Constraint for kpx motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);


        public static final ProfiledPIDController thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, kDThetaController, Constants.AutoConstants.kThetaControllerConstraints);

            
        
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
        
        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);
    }

    public static final class Trajectorys {
        //config to ensure the robot doesnt try to move through itself
        public static final  TrajectoryConfig config = new TrajectoryConfig(
                    TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
                    //TODO: Fix this to be the actual constant. 
                    2)
                .setKinematics(Constants.Swerve.swerveKinematics);

        public final static Trajectory onePiece =
                TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0,0, new Rotation2d(Units.degreesToRadians(0))),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(1,0)),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(2, 0, new Rotation2d(Units.degreesToRadians(0))),
                    config);
                
                    public final static Trajectory onePieceReef = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                    new Pose2d(2, 0, new Rotation2d(Units.degreesToRadians(0))),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(2.489185116263105,0)),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(3.25, 0, new Rotation2d(Units.degreesToRadians(0))),
                    config);

                    public final static Trajectory backup = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                    new Pose2d(3.25, 0, new Rotation2d(Units.degreesToRadians(0))),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(2.489185116263105,0)),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(2, 0, new Rotation2d(Units.degreesToRadians(0))),
                    config);
                    
        


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
                new Pose2d(0,5, new Rotation2d(Units.degreesToRadians(180))),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-1,3), new Translation2d(1, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 0, new Rotation2d(0)),
                config);
    }
    public class ClimberConstants {
        public static final int LEFT_SPARK_ID = 24;
        public static final int RIGHT_SPARK_ID = 23;
    }
 
    public static final int CANdleID = 19;
    

}
