// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
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
import frc.robot.Constants.ManipulatorConstants.IntakeConstants;
import frc.robot.Constants.ManipulatorConstants.PivotConstants;
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
	/*	Subsystems */
	private Climber climber = new Climber();
	private Elevator elevator = new Elevator(); 
	private Manipulator manipulator = new Manipulator();
	private CANdleSystem candle = new CANdleSystem(); 
	
	/*	Cameras / Vision */
    private static final PhotonCamera FrontCam = new PhotonCamera("Temp");
    private static final PhotonCamera FrontRight = new PhotonCamera("FrontRight");

    public static PhotonCamera getFrontRight() {
        return FrontRight;
    }

	/*	Autonomous Selector */
	private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Chooser");

    /*	Swerve */
	// Setting up bindings for necessary control of the swerve drive platform 
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Swerve.MaxSpeed * 0.01)
            .withRotationalDeadband(Constants.Swerve.MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(FrontCam, FrontRight);

	/*	Xbox Controllers */
    private final static CommandXboxController driver = Controls.driver;
    private final CommandXboxController operator = Controls.operator;

	/*	Logging */
    public final Telemetry logger = new Telemetry(Constants.Swerve.MaxSpeed);

	/*	Command Triggers */
	//		The robot starts tipping
	private Trigger tiltingElevator = new Trigger(() -> Math.abs(drivetrain.getPitch()) > 25); 
	//		Intake resistance spikes current, Robot has Algea
	private Trigger hasAlgea = new Trigger(manipulator.isCurrentSpiked());
	//		Beam Break triggers, Robot has Coral
	//private Trigger hasCoral = new Trigger(()-> manipulator.getBeamBreak()).debounce(.1);


    public RobotContainer() {
        configureBindings();
        chooseAuto();
    }

    private void configureBindings() {
		/***	Driver Controller ***/

		/**	Swerve Control 	**/
		SwerveUtils.setupUtil();

        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
			// 	Note that X is defined as forward according to WPILib convention,
        	// 	and Y is defined as to the left according to WPILib convention.
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-driver.getLeftY() * Constants.Swerve.MaxSpeed * elevator.getSpeedModifier()) 
                        .withVelocityY(-driver.getLeftX() * Constants.Swerve.MaxSpeed * elevator.getSpeedModifier())                                                                          
                        .withRotationalRate(
                                SwerveUtils.rotationPoint(new Rotation2d(
										-driver.getRightY(), 
										-driver.getRightX()).getDegrees(), 
										drivetrain.getYaw()) * Constants.Swerve.MaxAngularRate * elevator.getSpeedModifier()) 
                .withDesaturateWheelSpeeds(true)
		));
		// 			Reset the field-centric heading on left bumper press
		driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

		/**		LEDs and Autimation 	**/
		//			Climb LED sequence
		driver.povUp().onTrue(candle.runOnce(() -> {
            candle.clearAllAnims();
            candle.changeAnimation(AnimationTypes.Climb);
        }));
		//			Allignment To Reef Left Branch 
        driver.x().whileTrue(new SequentialCommandGroup(
                candle.setLights(AnimationTypes.SingleFade),
                alignAndDriveToReef(Units.inchesToMeters(VisionConstants.LEFT_REEF_OFFSET)),
                candle.setLights(AnimationTypes.Align)
		));
		//			Allignment To Reef Right Branch
        driver.b().whileTrue(new SequentialCommandGroup(
                candle.setLights(AnimationTypes.SingleFade),
                alignAndDriveToReef(Units.inchesToMeters(VisionConstants.RIGHT_REEF_OFFSET)),
                candle.setLights(AnimationTypes.Align)
		));

        driver.a().whileTrue(alignAndDriveToSource(Units.inchesToMeters(0)));
		drivetrain.registerTelemetry(logger::telemeterize);

		/***	Operator Controller ***/

		/**		Scoring Positions **/
		//			L2 Coral
        operator.a().onTrue(new ParallelCommandGroup(
                elevator.setPosition(
                        ElevatorConstants.L2),
                manipulator.setPosition(
                        IntakeConstants.HOLD_CORAL,
                        PivotConstants.PLACE_CORAL)));
		//			L3 Coral
        operator.b().onTrue(new ParallelCommandGroup(
                elevator.setPosition(
                        ElevatorConstants.L3),
                manipulator.setPosition(
                        IntakeConstants.HOLD_CORAL,
                        PivotConstants.PLACE_CORAL)
		));
		//			L4 Coral
		operator.y().onTrue(new ParallelCommandGroup(
                elevator.setPosition(
                        ElevatorConstants.L4),
                manipulator.setPosition(
                        IntakeConstants.HOLD_CORAL,
                        PivotConstants.L4)
		));
		//			Processor Algea
		operator.povDown().onTrue(new ParallelCommandGroup(
				elevator.setPosition(ElevatorConstants.PROCESSOR),
				manipulator.setPosition(
						IntakeConstants.HOLD_CORAL,
						PivotConstants.STRAIGHTOUT)
		));
		//			Barge Algea
        operator.povUp().onTrue(new ParallelCommandGroup(
                elevator.setPosition(ElevatorConstants.BARGE),
                manipulator.setPosition(
                        IntakeConstants.HOLD_ALGEA,
                        PivotConstants.BARGE),
                candle.setLights(AnimationTypes.Rainbow)
		));		

		/**		Pick-Up / Hold Positions **/
		//			Intaking
		operator.leftTrigger(.01).whileTrue(
                new ParallelCommandGroup(
                        manipulator.setVelocity(()->IntakeConstants.INTAKE_SPEED),
                        candle.setLights(AnimationTypes.Looking)
		));
		//			Expelling
		operator.rightTrigger(.01).whileTrue(
			new SequentialCommandGroup(
					manipulator.expelCommand(elevator),
					manipulator.setClaw(IntakeConstants.HOLD_ALGEA),
					candle.setLights(AnimationTypes.Looking)
		));

        operator.leftBumper().onTrue(manipulator.stopIntake());

		//			Station Pick-Up Coral
        operator.x().onTrue(new ParallelCommandGroup(
                elevator.setPosition(
                        ElevatorConstants.STATION),
                new SequentialCommandGroup(
                        manipulator.setPosition(
                                IntakeConstants.HOLD_CORAL,
                                PivotConstants.STATION),
                        manipulator.setVelocity(() -> IntakeConstants.INTAKE_SPEED)),
                candle.setLights(AnimationTypes.Looking)
		));
		//			Floor Pick-Up Algea
		operator.start().onTrue(new ParallelCommandGroup(
			elevator.setPosition(ElevatorConstants.FLOOR),
			manipulator.setPosition(
					IntakeConstants.HOLD_ALGEA,
					PivotConstants.FLOOR_PICKUP)
		));
		//			L2 Reef Algea
		operator.povLeft().onTrue(
			new ParallelCommandGroup(
					manipulator.setPosition(
							IntakeConstants.HOLD_ALGEA,
							PivotConstants.STRAIGHTOUT),
					elevator.setPosition(ElevatorConstants.L2 - 6)
		));
		//			L3 Reef Algea
		operator.povRight().onTrue(
			new ParallelCommandGroup(
					manipulator.setPosition(
							IntakeConstants.HOLD_ALGEA,
							PivotConstants.STRAIGHTOUT),
					elevator.setPosition(ElevatorConstants.L3 - 7)
		));
		
		/**		Climbing **/

		climber.setDefaultCommand(climber.climb(()->operator.getLeftY()));

		operator.rightStick().whileTrue(climber.climb(()-> 1).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

		/***	Automation ***/
		//		Lowers elevator if tipping
        tiltingElevator.onTrue(elevator.setPosition(ElevatorConstants.PROCESSOR));

		//		Lifts manipulator after intaking algea
		//			LED change for having algea
        hasAlgea.onTrue(new ParallelCommandGroup(
                manipulator.setPosition(
                        IntakeConstants.HOLD_CORAL,
                        PivotConstants.STRAIGHTOUT),
                
                candle.setLights(AnimationTypes.HasAlgea)
		));

		/***	Unused Code ***/

		//	Automates stopping Intake with coral
		//		LED change for having coral
		// hasCoral.onTrue(
        //         new SequentialCommandGroup(
        //         Commands.waitSeconds(.05),
        //         new ParallelCommandGroup(
        //                 manipulator.stopIntake(),
        //                 candle.setLights(AnimationTypes.HasCoral)
        //         )).unless(hasAlgea));

		//	Drivetrain breaking/slowing
        // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
		
		/*	Sys ID: Drivetrain PID values */
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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

        autoChooser.addOption("1c1a blue mid", new MiddleOnePieceBlue(drivetrain,elevator,manipulator).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        autoChooser.addOption("1c1a red mid", new MiddleOnePieceRed(drivetrain,elevator,manipulator).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        autoChooser.addOption("1c1a blue side", new SideOnePieceBlue(drivetrain,elevator,manipulator).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        autoChooser.addOption("1c1a red side", new SideOnePieceRed(drivetrain,elevator,manipulator).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        autoChooser.addOption("Just Coral red side", new JustCoralRed(drivetrain,elevator,manipulator).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        autoChooser.addOption("Just Coral Blue side", new JustCoralBlue(drivetrain,elevator,manipulator).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

		autoChooser.addDefaultOption("Do Nothing", elevator.setPosition(ElevatorConstants.PROCESSOR));

    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

}
