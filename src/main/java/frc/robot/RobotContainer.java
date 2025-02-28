// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.AlignToTag;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.InfeedConstants.IntakeConstants;
import frc.robot.Constants.InfeedConstants.PivotConstants;
import frc.robot.Constants.Trajectorys;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.CANdleSystem.AnimationTypes;
import frc.robot.subsystems.CANdleSystem;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private PhotonCamera frontLeftCam = new PhotonCamera("FrontLeft");
    private PhotonCamera frontRightCam = new PhotonCamera("FrontRight");


    Elevator elevator = new Elevator();
    Manipulator maniuplator = new Manipulator();
    CANdleSystem candle = new CANdleSystem();

// Construct PhotonPoseEstimator
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = Controls.driver;

    private final CommandXboxController operator = Controls.operator;


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );


        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
       // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
       // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
       // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
       // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driver.a().whileTrue(new AlignToTag(drivetrain, frontRightCam, frontLeftCam, ()->driver.getLeftX()));

        driver.povDown().onTrue(candle.runOnce(()->{
            candle.clearAllAnims(); 
            candle.incrementAnimation();
        }));


        driver.povUp().onTrue(candle.runOnce(()->{ 
            candle.clearAllAnims();
            candle.changeAnimation(AnimationTypes.Strobe);
        }));
        

        operator.y().onTrue(new ParallelCommandGroup(
            elevator.setPosition(
                ElevatorConstants.L4),
            maniuplator.setPosition(
                IntakeConstants.HOLD_CORAL,
                PivotConstants.L4)
            ));
        operator.b().onTrue(new ParallelCommandGroup(
            elevator.setPosition(
                ElevatorConstants.L3),
            maniuplator.setPosition(
                IntakeConstants.HOLD_CORAL,
                PivotConstants.CLEAR_ALGEA)
            ));
        operator.a().onTrue(new ParallelCommandGroup(
            elevator.setPosition(
                ElevatorConstants.L2),
            maniuplator.setPosition(
                IntakeConstants.HOLD_CORAL,
                PivotConstants.CLEAR_ALGEA)
            ));
        operator.x().onTrue(new ParallelCommandGroup(
            elevator.setPosition(
                ElevatorConstants.STATION),
            maniuplator.setPosition(
                IntakeConstants.HOLD_CORAL,
                PivotConstants.STATION)
            ));
        // Straightens out intake and position to hold coral
        operator.povLeft().onTrue(
            maniuplator.setPosition(
                IntakeConstants.HOLD_CORAL,
                PivotConstants.CLEAR_ALGEA)
            );
        //  Straightens out intake
        operator.povRight().onTrue(
            maniuplator.setPosition(
                IntakeConstants.HOLD_ALGEA, 
                PivotConstants.CLEAR_ALGEA)
            );
        //  Sucks in piece
        operator.leftTrigger(.00).whileTrue(
            maniuplator.setVelocity(()->operator.getLeftTriggerAxis())
            );
        //  Repels piece in intake
        operator.rightTrigger(.00).whileTrue(
            maniuplator.setVelocity(()->-operator.getRightTriggerAxis())
            );
        // Moves Elevator Up to score in barge
        operator.povUp().onTrue(new ParallelCommandGroup(
            elevator.setPosition(ElevatorConstants.BARGE), 
            maniuplator.setPosition(
                IntakeConstants.HOLD_ALGEA, 
                PivotConstants.BARGE)
            ));
        // Moves Elevator Down
        operator.povDown().onTrue(new ParallelCommandGroup(
            elevator.setPosition(ElevatorConstants.PROCESSOR), 
            maniuplator.setPosition(
                IntakeConstants.HOLD_CORAL, 
                PivotConstants.CLEAR_ALGEA)
            ));

        drivetrain.registerTelemetry(logger::telemeterize);
    }



    public Command getAutonomousCommand() {
        return drivetrain.getAutoCommand(Trajectorys.onePiece);
    }

    
}

