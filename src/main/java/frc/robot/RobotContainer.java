// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AlignAndDriveToReef;
import frc.robot.Commands.AlignToTag;
import frc.robot.Commands.Autos.JustCoralBlue;
import frc.robot.Commands.Autos.JustCoralRed;
import frc.robot.Commands.Autos.MiddleOnePieceBlue;
import frc.robot.Commands.Autos.MiddleOnePieceRed;
import frc.robot.Commands.Autos.SideOnePieceBlue;
import frc.robot.Commands.Autos.SideOnePieceRed;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.InfeedConstants.IntakeConstants;
import frc.robot.Constants.InfeedConstants.PivotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utils.SwerveUtils;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CANdleSystem.AnimationTypes;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

public final class RobotContainer {

    private static final PhotonCamera FrontCam = new PhotonCamera("Temp");
    private static final PhotonCamera FrontRight = new PhotonCamera("FrontRight");


    

    public static PhotonCamera getFrontRight() {
        return FrontRight;
    }

    Elevator elevator = new Elevator();
    Manipulator maniuplator = new Manipulator();
    CANdleSystem candle = new CANdleSystem();

    private final SendableChooser<Command> chooser = new SendableChooser<>();

    Climber climber = new Climber();
    

    
    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
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
        SwerveUtils.setupUtil();
        Trigger tiltingElevator = new Trigger(() -> Math.abs(drivetrain.getPitch()) > 25);
        Trigger hasAlgea = new Trigger(maniuplator.isCurrentSpiked());

        Trigger hasCoral = new Trigger(()-> maniuplator.getBeamBreak()).debounce(.1);
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-driver.getLeftY() * Constants.Swerve.MaxSpeed * elevator.getSpeedModifier()) 
                        .withVelocityY(-driver.getLeftX() * Constants.Swerve.MaxSpeed * elevator.getSpeedModifier())                                                                          
                        .withRotationalRate(
                                -driver.getRightX() * Constants.Swerve.MaxAngularRate * elevator.getSpeedModifier())
                                //SwerveUtils.rotationPoint(new Rotation2d(-driver.getRightY(), -driver.getRightX()).getDegrees(), drivetrain.getYaw()) * Constants.Swerve.MaxAngularRate * elevator.getSpeedModifier()) 
                .withDesaturateWheelSpeeds(true)));

        tiltingElevator.onTrue(elevator.setPosition(ElevatorConstants.PROCESSOR));

        hasAlgea.onTrue(new ParallelCommandGroup(
                maniuplator.setPosition(
                        IntakeConstants.HOLD_CORAL,
                        PivotConstants.STRAIGHTOUT),
                
                candle.setLights(AnimationTypes.HasAlgea)));

        // hasCoral.onTrue(
        //         new SequentialCommandGroup(
        //         Commands.waitSeconds(.05),
        //         new ParallelCommandGroup(
        //                 maniuplator.stopIntake(),
        //                 candle.setLights(AnimationTypes.HasCoral)
        //         )).unless(hasAlgea));



        // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driver.povUp().onTrue(candle.runOnce(() -> {
            candle.clearAllAnims();
            candle.changeAnimation(AnimationTypes.Climb);
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
                        maniuplator.setVelocity(() -> IntakeConstants.INTAKE_SPEED)),
                candle.setLights(AnimationTypes.Looking)));

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
                        elevator.setPosition(ElevatorConstants.L3 - 7)));

        // Sucks in piece
        operator.leftTrigger(.01).whileTrue(
                new ParallelCommandGroup(
                        maniuplator.setVelocity(()->IntakeConstants.INTAKE_SPEED),
                        candle.setLights(AnimationTypes.Looking)));

        // Repels piece in intake
        operator.rightTrigger(.01).whileTrue(
                new SequentialCommandGroup(
                        maniuplator.expelCommand(elevator),
                        maniuplator.setClaw(IntakeConstants.HOLD_ALGEA),
                        candle.setLights(AnimationTypes.Looking)));

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
                        PivotConstants.STRAIGHTOUT)
        ));

        operator.start().onTrue(new ParallelCommandGroup(
                elevator.setPosition(ElevatorConstants.FLOOR),
                maniuplator.setPosition(
                        IntakeConstants.HOLD_ALGEA,
                        PivotConstants.FLOOR_PICKUP + 2)));

        operator.leftBumper().onTrue(maniuplator.stopIntake());

        climber.setDefaultCommand(climber.climb(()->operator.getLeftY()));

        drivetrain.registerTelemetry(logger::telemeterize);

        driver.x().whileTrue(new SequentialCommandGroup(
                candle.setLights(AnimationTypes.SingleFade),
                alignAndDriveToReef(Units.inchesToMeters(-2.6)),
                candle.setLights(AnimationTypes.Align)));

        driver.b().whileTrue(new SequentialCommandGroup(
                candle.setLights(AnimationTypes.SingleFade),
                alignAndDriveToReef(Units.inchesToMeters(2.6)),
                candle.setLights(AnimationTypes.Align)));

        driver.a().whileTrue(alignAndDriveToSource(Units.inchesToMeters(0)));

        operator.rightStick().whileTrue(climber.climb(()-> 1).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));


    }

    // Automatically chooses closest tag
    public static Command alignAndDriveToReef(double offset) {
        return Commands.defer(
                () -> {
                    Pose2d alignmentPose = drivetrain.findNearestReefTagPose()
                            .plus(
                                    new Transform2d(
                                            new Translation2d(VisionConstants.TELEOP_REEF_DISTANCE , offset),
                                            new Rotation2d()));
                    return new AlignAndDriveToReef(
                            drivetrain,
                            offset,
                            alignmentPose,
                            Rotation2d.kPi,
                            Units.inchesToMeters(2));
                },
                Set.of(drivetrain));
    }


    // Automatically chooses closest source
    public static Command alignAndDriveToSource(double offset) {
        return Commands.defer(
                () -> {
                    Pose2d alignmentPose = drivetrain.findNearestSourceTagPose()
                            .plus(
                                    new Transform2d(
                                            new Translation2d(VisionConstants.TELEOP_REEF_DISTANCE , offset),
                                            new Rotation2d()));
                    return new AlignAndDriveToReef(
                            drivetrain,
                            offset,
                            alignmentPose,
                            Rotation2d.kPi,
                            Units.inchesToMeters(.5));
                },
                Set.of(drivetrain));
    }


    public static Command alignAndDriveToReef(int tag, double offset, double distanceOffset) {
        return Commands.defer(
                () -> {
                    Pose2d alignmentPose = VisionConstants.aprilTagLayout.getTagPose(tag).get().toPose2d()
                            .plus(
                                    new Transform2d(
                                            new Translation2d(distanceOffset, offset),
                                            new Rotation2d()));
                    return new AlignAndDriveToReef(
                            drivetrain,
                            offset,
                            alignmentPose,
                            Rotation2d.kPi,
                            Units.inchesToMeters(1));
                },
                Set.of(drivetrain));
    }

    public static Command alignAndDrive(int tag, double offset, double distanceOffset) {
        return Commands.defer(
                () -> {
                    Pose2d alignmentPose = VisionConstants.aprilTagLayout.getTagPose(tag).get().toPose2d()
                            .plus(
                                    new Transform2d(
                                            new Translation2d(distanceOffset, offset),
                                            new Rotation2d()));
                    return new AlignAndDriveToReef(
                            drivetrain,
                            offset,
                            alignmentPose,
                            Rotation2d.kPi,
                            Units.inchesToMeters(3));
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

    public static Command alignToPiece(double offset) {
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

public static void endgameRumble(){
    if (DriverStation.isEnabled()){
        if (DriverStationJNI.getMatchTime() <= 25 && DriverStationJNI.getMatchTime() >= 1 ){
                Controls.xbox_operator.setRumble(RumbleType.kBothRumble, 1);
    }else {
      Controls.xbox_operator.setRumble(RumbleType.kBothRumble, 0);
    }
  }
}

    public void chooseAuto() {

        chooser.addOption("1c1a blue mid", new MiddleOnePieceBlue(drivetrain,elevator,maniuplator).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        chooser.addOption("1c1a red mid", new MiddleOnePieceRed(drivetrain,elevator,maniuplator).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        chooser.addOption("1c1a blue side", new SideOnePieceBlue(drivetrain,elevator,maniuplator).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        chooser.addOption("1c1a red side", new SideOnePieceRed(drivetrain,elevator,maniuplator).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        chooser.addOption("Just Coral red side", new JustCoralRed(drivetrain,elevator,maniuplator).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        chooser.addOption("Just Coral Blue side", new JustCoralBlue(drivetrain,elevator,maniuplator).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
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
