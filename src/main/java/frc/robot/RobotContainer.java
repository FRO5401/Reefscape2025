// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import org.photonvision.PhotonCamera;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AlignAndDriveToReef;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.InfeedConstants.IntakeConstants;
import frc.robot.Constants.InfeedConstants.PivotConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.CANdleSystem.AnimationTypes;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.Commands.AlignToTag;

public class RobotContainer {

    private static PhotonCamera FrontCam = new PhotonCamera("Temp");
    private static PhotonCamera FrontRight = new PhotonCamera("FrontRight");

    Elevator elevator = new Elevator();
    Manipulator maniuplator = new Manipulator();
    CANdleSystem candle = new CANdleSystem();

    private final SendableChooser<Command> chooser = new SendableChooser<>();

    Climber climber = new Climber();
    

    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Swerve.MaxSpeed * 0.01)
            .withRotationalDeadband(Constants.Swerve.MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final static CommandXboxController driver = Controls.driver;
    private final CommandXboxController operator = Controls.operator;
    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(FrontCam, FrontRight);

    public final Telemetry logger = new Telemetry(Constants.Swerve.MaxSpeed);

    public RobotContainer() {
        configureBindings();
        chooseAuto();
    }

    private void configureBindings() {
        Trigger tiltingElevator = new Trigger(() -> drivetrain.getPitch() > 25);
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-driver.getLeftY() * Constants.Swerve.MaxSpeed * elevator.getSpeedModifier()) // Drive
                                                                                                                     // forward
                                                                                                                     // with
                                                                                                                     // negative
                                                                                                                     // Y
                                                                                                                     // (forward)
                        .withVelocityY(-driver.getLeftX() * Constants.Swerve.MaxSpeed * elevator.getSpeedModifier()) // Drive
                                                                                                                     // left
                                                                                                                     // with
                                                                                                                     // negative
                                                                                                                     // X
                                                                                                                     // (left)
                        .withRotationalRate(
                                -driver.getRightX() * Constants.Swerve.MaxAngularRate * elevator.getSpeedModifier()) // Drive
                                                                                                                     // counterclockwise
                                                                                                                     // with
                                                                                                                     // negative
                                                                                                                     // X
                                                                                                                     // (left)
                ));

        tiltingElevator.onTrue(elevator.setPosition(ElevatorConstants.PROCESSOR));

        // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // driver.a().whileTrue(new AlignToTag(drivetrain, frontRightCam, frontLeftCam,
        // ()->driver.getLeftX()));

        driver.povDown().onTrue(candle.runOnce(() -> {
            candle.clearAllAnims();
            candle.incrementAnimation();
        }));

        driver.povUp().onTrue(candle.runOnce(() -> {
            candle.clearAllAnims();
            candle.changeAnimation(AnimationTypes.Strobe);
        }));

        operator.y().onTrue(new ParallelCommandGroup(
                elevator.setPosition(
                        ElevatorConstants.L4),
                maniuplator.setPosition(
                        IntakeConstants.HOLD_CORAL,
                        PivotConstants.L4)));
        operator.b().onTrue(new ParallelCommandGroup(
                elevator.setPosition(
                        ElevatorConstants.L3),
                maniuplator.setPosition(
                        IntakeConstants.HOLD_CORAL,
                        PivotConstants.PLACE_CORAL)));
        operator.a().onTrue(new ParallelCommandGroup(
                elevator.setPosition(
                        ElevatorConstants.L2),
                maniuplator.setPosition(
                        IntakeConstants.HOLD_CORAL,
                        PivotConstants.PLACE_CORAL)));

        operator.x().onTrue(new ParallelCommandGroup(
                elevator.setPosition(
                        ElevatorConstants.STATION),
                new SequentialCommandGroup(
                        maniuplator.setPosition(
                                IntakeConstants.HOLD_CORAL,
                                PivotConstants.STATION),
                        maniuplator.setVelocity(() -> 1))));

        // Straightens out intake and position to hold coral
        operator.povLeft().onTrue(
                new ParallelCommandGroup(
                        maniuplator.setPosition(

                                IntakeConstants.HOLD_ALGEA,
                                PivotConstants.STRAIGHTOUT),
                        elevator.setPosition(ElevatorConstants.L2 - 6)));

        // Straightens out intake
        operator.povRight().onTrue(
                new ParallelCommandGroup(
                        maniuplator.setPosition(
                                IntakeConstants.HOLD_ALGEA,
                                PivotConstants.STRAIGHTOUT),
                        elevator.setPosition(ElevatorConstants.L3 - 6)));

        // Sucks in piece
        operator.leftTrigger(.01).whileTrue(
                maniuplator.setVelocity(() -> 1));

        // Repels piece in intake
        operator.rightTrigger(.01).whileTrue(
                new SequentialCommandGroup(maniuplator.setVelocity(() -> -1),
                        maniuplator.setClaw(IntakeConstants.HOLD_ALGEA)));

        // Moves Elevator Up to score in barge
        operator.povUp().onTrue(new ParallelCommandGroup(
                elevator.setPosition(ElevatorConstants.BARGE),
                maniuplator.setPosition(
                        IntakeConstants.HOLD_ALGEA,
                        PivotConstants.BARGE),
                candle.setLights(AnimationTypes.Rainbow)));

        // Moves Elevator Down
        operator.povDown().onTrue(new ParallelCommandGroup(
                elevator.setPosition(ElevatorConstants.PROCESSOR),
                maniuplator.setPosition(
                        IntakeConstants.HOLD_CORAL,
                        PivotConstants.CLEAR_ALGEA),
                candle.setLights(AnimationTypes.SingleFade)));

        operator.start().onTrue(new ParallelCommandGroup(
                elevator.setPosition(ElevatorConstants.FLOOR),
                maniuplator.setPosition(
                        IntakeConstants.HOLD_ALGEA,
                        PivotConstants.FLOOR_PICKUP + 2)));

        operator.leftBumper().onTrue(maniuplator.stopIntake());

        climber.setDefaultCommand(climber.climb(()->operator.getLeftY()));

        drivetrain.registerTelemetry(logger::telemeterize);
        driver.x().whileTrue(alignAndDriveToReef(Units.inchesToMeters(-2.5)));
        driver.b().whileTrue(alignAndDriveToReef(Units.inchesToMeters(2.5)));
        driver.a().whileTrue(alignToSource(Units.inchesToMeters(0)));



    }

    // Automatically chooses closest tag
    public static Command alignAndDriveToReef(double offset) {
        return Commands.defer(
                () -> {
                    Pose2d alignmentPose = drivetrain.findNearestReefTagPose()
                            .plus(
                                    new Transform2d(
                                            new Translation2d(VisionConstants.REEF_DISTANCE, offset),
                                            new Rotation2d()));
                    return new AlignAndDriveToReef(
                            drivetrain,
                            offset,
                            alignmentPose,
                            Rotation2d.kPi);
                },
                Set.of(drivetrain));
    }

    public static Command alignToReef(double offset) {
        return Commands.defer(
                () -> {
                    Pose2d alignmentPose = drivetrain.findNearestReefTagPose();
                    return new AlignToTag(
                            drivetrain,
                            () -> -driver.getLeftY(),
                            () -> driver.getLeftX(),
                            offset,
                            alignmentPose,
                            Rotation2d.kPi.plus(new Rotation2d()));
                },
                Set.of(drivetrain));
    }

    public static Command alignToSource(double offset) {
        return Commands.defer(
                () -> {
                    Pose2d alignmentPose = drivetrain.findNearestSourceTagPose();
                    return new AlignToTag(
                            drivetrain,
                            () -> driver.getLeftY(),
                            () -> driver.getLeftX(),
                            offset,
                            alignmentPose,
                            Rotation2d.kPi.plus(new Rotation2d()));
                },
                Set.of(drivetrain));
    }

    public void chooseAuto() {

        // chooser.setDefaultOption("One Piece", new
        // OnePiece(drivetrain,elevator,maniuplator).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        // chooser.addOption("Move Forward",
        // drivetrain.getAutoCommand(Trajectorys.onePiece));
        chooser.setDefaultOption("Do Nothing", elevator.setPosition(ElevatorConstants.PROCESSOR));

        Shuffleboard.getTab("Autonomous").add(chooser);
        SmartDashboard.putData(chooser);

    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

}
