// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import org.photonvision.PhotonCamera;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AlignAndDriveToReef;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.InfeedConstants.IntakeConstants;
import frc.robot.Constants.InfeedConstants.PivotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.CANdleSystem.AnimationTypes;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.Commands.AlignToReef;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private PhotonCamera FrontCam = new PhotonCamera("FrontCam");


    Elevator elevator = new Elevator();
    Manipulator maniuplator = new Manipulator();
    CANdleSystem candle = new CANdleSystem();

    private final SendableChooser<Command> chooser = new SendableChooser<>();

    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    private final CommandXboxController driver = Controls.driver;

    private final CommandXboxController operator = Controls.operator;


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(FrontCam);

    public RobotContainer() {
        configureBindings();
        chooseAuto();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed * elevator.getSpeedModifier()) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed * elevator.getSpeedModifier()) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate * elevator.getSpeedModifier()) // Drive counterclockwise with negative X (left)
            )
        );


        //driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
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

        //driver.a().whileTrue(new AlignToTag(drivetrain, frontRightCam, frontLeftCam, ()->driver.getLeftX()));

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
                PivotConstants.PLACE_CORAL)
            ));
        operator.a().onTrue(new ParallelCommandGroup(
            elevator.setPosition(
                ElevatorConstants.L2),
            maniuplator.setPosition(
                IntakeConstants.HOLD_CORAL,
                PivotConstants.PLACE_CORAL)
            ));

        operator.x().onTrue(new ParallelCommandGroup(
            elevator.setPosition(
                ElevatorConstants.STATION),
            new SequentialCommandGroup(
            maniuplator.setPosition(
                IntakeConstants.HOLD_CORAL,
                PivotConstants.STATION),
            maniuplator.setVelocity(()-> 1))
            ));
    
        // Straightens out intake and position to hold coral
        operator.povLeft().onTrue(
            new ParallelCommandGroup(
                maniuplator.setPosition(
               
                IntakeConstants.HOLD_ALGEA,
                    PivotConstants.STRAIGHTOUT),
                elevator.setPosition(ElevatorConstants.L2-5)
            ));
        
        //  Straightens out intake
        operator.povRight().onTrue(
            new ParallelCommandGroup(
                maniuplator.setPosition(
                    IntakeConstants.HOLD_ALGEA,
                    PivotConstants.STRAIGHTOUT),
                elevator.setPosition(ElevatorConstants.L3-5)
            ));
            
        //  Sucks in piece
        operator.leftTrigger(.01).whileTrue(
            maniuplator.setVelocity(()->1)
            );

        //  Repels piece in intake
        operator.rightTrigger(.01).whileTrue(
            new SequentialCommandGroup(maniuplator.setVelocity(()->-1), maniuplator.setClaw(IntakeConstants.HOLD_ALGEA)));

        // Moves Elevator Up to score in barge
        operator.povUp().onTrue(new ParallelCommandGroup(
            elevator.setPosition(ElevatorConstants.BARGE), 
            maniuplator.setPosition(
                IntakeConstants.HOLD_ALGEA, 
                PivotConstants.BARGE),
            candle.setLights(AnimationTypes.Rainbow)
            ));
        // Moves Elevator Down
        operator.povDown().onTrue(new ParallelCommandGroup(
            elevator.setPosition(ElevatorConstants.PROCESSOR), 
            maniuplator.setPosition(
                IntakeConstants.HOLD_CORAL, 
                PivotConstants.CLEAR_ALGEA),
            candle.setLights(AnimationTypes.SingleFade)
            )); 
             
        operator.start().onTrue(new ParallelCommandGroup(
            elevator.setPosition(ElevatorConstants.FLOOR), 
            maniuplator.setPosition(
                IntakeConstants.HOLD_ALGEA, 
                PivotConstants.PLACE_CORAL+2)
            ));
        

        operator.leftBumper().onTrue(maniuplator.stopIntake());
       // drivetrain.registerTelemetry(logger::telemeterize);

       driver.a().whileTrue(alignToReef(0));

}

    // Automatically chooses closest tag
    public Command alignAndDriveToReef(double offset) {
        return Commands.defer(
                () -> {
                    Pose2d alignmentPose = drivetrain.findNearestAprilTagPose()                      
                    .plus(
                        new Transform2d(
                                new Translation2d(offset, VisionConstants.REEF_DISTANCE),
                                new Rotation2d()));
                    return new AlignAndDriveToReef(
                            drivetrain,
                            offset,
                            alignmentPose,
                            Rotation2d.kPi);
                },
                Set.of(drivetrain));
    }

    public Command alignToReef(double offset) {
        return Commands.defer(
            () -> {
                Pose2d alignmentPose = drivetrain.findNearestAprilTagPose();
                return new AlignToReef(
                        drivetrain,
                        ()->driver.getLeftX(),
                        ()->driver.getLeftY(),
                        offset,
                        alignmentPose,
                        Rotation2d.kPi);
                },
                Set.of(drivetrain));
    }



    public void chooseAuto(){
        
    //chooser.setDefaultOption("One Piece", new OnePiece(drivetrain,elevator,maniuplator).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    //chooser.addOption("Move Forward", drivetrain.getAutoCommand(Trajectorys.onePiece));
    chooser.setDefaultOption("Do Nothing", elevator.setPosition(ElevatorConstants.PROCESSOR));

    Shuffleboard.getTab("Autonomous").add(chooser);
    SmartDashboard.putData(chooser);

  }


    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}   
// /*

// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.Constants.ModeConstants;
// import frc.robot.Constants.ElevatorConstants.ElevatorSimConstants;
// import frc.robot.commands.DriveCommands;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.drive.GyroIO;
// import frc.robot.subsystems.drive.GyroIOPigeon2;
// import frc.robot.subsystems.drive.ModuleIO;
// import frc.robot.subsystems.drive.ModuleIOSim;
// import frc.robot.subsystems.drive.ModuleIOTalonFX;
// import frc.robot.subsystems.elevator.Elevator;
// import frc.robot.subsystems.elevator.ElevatorSimulation;

// /**
//  * This class is where the bulk of the robot should be declared. Since Command-based is a
//  * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
//  * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
//  * subsystems, commands, and button mappings) should be declared here.
//  */
// public class RobotContainer {
//   // Subsystems
//   private final Drive drive;
//   // Controller
//   private final CommandXboxController driver = Controls.driver;
//   private final CommandXboxController operator = Controls.operator;
  
//   private final Encoder elevatorEncoder = new Encoder(ElevatorSimConstants.kEncoderAChannel, ElevatorSimConstants.kEncoderBChannel);
//     private final PWMSparkMax elevatorMotor = new PWMSparkMax(ElevatorConstants.elevatorID);

//     private final Elevator m_elevator = new Elevator(elevatorEncoder, elevatorMotor);

//     private ElevatorSimulation m_elevatorSimulation;
  
//   /** The container for the robot. Contains subsystems, OI devices, and commands. */
//   public RobotContainer() {
//     switch (ModeConstants.currentMode) {
//       case REAL:
//         // Real robot, instantiate hardware IO implementations
//         drive =
//             new Drive(
//                 new GyroIOPigeon2(),
//                 new ModuleIOTalonFX(TunerConstants.FrontLeft),
//                 new ModuleIOTalonFX(TunerConstants.FrontRight),
//                 new ModuleIOTalonFX(TunerConstants.BackLeft),
//                 new ModuleIOTalonFX(TunerConstants.BackRight));
//         break;

//       case SIM:
//         // Sim robot, instantiate physics sim IO implementations
//         drive =
//             new Drive(
//                 new GyroIO() {},
//                 new ModuleIOSim(TunerConstants.FrontLeft),
//                 new ModuleIOSim(TunerConstants.FrontRight),
//                 new ModuleIOSim(TunerConstants.BackLeft),
//                 new ModuleIOSim(TunerConstants.BackRight));
        
//         m_elevatorSimulation = new ElevatorSimulation(elevatorEncoder, elevatorMotor);
//         break;

//       default:
//         // Replayed robot, disable IO implementations
//         drive =
//             new Drive(
//                 new GyroIO() {},
//                 new ModuleIO() {},
//                 new ModuleIO() {},
//                 new ModuleIO() {},
//                 new ModuleIO() {});
//         break;
//     }
//     CommandScheduler.getInstance().setDefaultCommand(m_elevator, m_elevator.depower());

//     // Configure the button bindings
//     configureButtonBindings();
//   }

//   private void configureButtonBindings() {
//     // Default command, normal field-relative drive
//     drive.setDefaultCommand(
//         DriveCommands.joystickDrive(
//             drive,
//             () -> -driver.getLeftY() * ModeConstants.flipped(),
//             () -> -driver.getLeftX() * ModeConstants.flipped(),
//             () -> -driver.getRawAxis(5) * ModeConstants.flipped()));

//     // Lock to 0° when A button is held
//     driver
//         .a()
//         .whileTrue(
//             DriveCommands.joystickDriveAtAngle(
//                 drive,
//                 () -> -driver.getLeftY() * ModeConstants.flipped(),
//                 () -> -driver.getLeftX() * ModeConstants.flipped(),
//                 () -> new Rotation2d()));

//     // Switch to X pattern when X button is pressed
//     driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

//     // Reset gyro to 0° when B button is pressed
//     driver
//         .b()
//         .onTrue(
//             Commands.runOnce(
//                     () ->
//                         drive.setPose(
//                             new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
//                     drive)
//                 .ignoringDisable(true));
    
//     new Trigger(()->{return RobotState.isEnabled() && RobotState.isTeleop();}).onTrue(m_elevator.initConstants());

//     operator.a().onTrue(m_elevator.goToHeight(0.25));
//     operator.b().onTrue(m_elevator.goToHeight(0.50));
//     operator.y().onTrue(m_elevator.goToHeight(0.75));
//     operator.x().onTrue(m_elevator.goToHeight(1.00));
//     operator.povDown().onTrue(m_elevator.goToHeight(0.00));
        
//     operator.start().whileTrue(m_elevator.manualControl(()->-0.2*operator.getLeftY()));
//     operator.back().whileTrue(m_elevator.closedLoopManualSetpoint(()->-1.25*operator.getLeftY()));

//   }
//   public void simulationPeriodic()
//   {
//       m_elevatorSimulation.elevatorSimulationPeriodic();
//   }

//   public void close() 
//     {
//         elevatorEncoder.close();
//         elevatorMotor.close();
//         m_elevator.close();
//     }

//   public Command getAutonomousCommand() {
//     return null;
//   }

// }
// */