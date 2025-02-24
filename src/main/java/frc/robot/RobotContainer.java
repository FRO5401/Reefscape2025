// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.InfeedConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.CANdleSystem.AnimationTypes;
import frc.robot.subsystems.CANdleSystem;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    Elevator elevator = new Elevator();
    Manipulator maniuplator = new Manipulator();
    CANdleSystem candle = new CANdleSystem();
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandXboxController Operator = new CommandXboxController(1);


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
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );


        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
       // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
       // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
       // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
       // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.povDown().onTrue(candle.runOnce(()->{
            candle.clearAllAnims(); 
            candle.incrementAnimation();
        }));


        joystick.povUp().onTrue(candle.runOnce(()->{ 
            candle.clearAllAnims();
            candle.changeAnimation(AnimationTypes.Strobe);
        }));
        

        Operator.y().onTrue(elevator.setPosition(Constants.ElevatorConstants.L4));
        Operator.b().onTrue(elevator.setPosition(Constants.ElevatorConstants.L3));
        Operator.a().onTrue(elevator.setPosition(Constants.ElevatorConstants.L2));
        Operator.x().onTrue(elevator.setPosition(Constants.ElevatorConstants.STATION));
        Operator.povUp().onTrue(new ParallelCommandGroup(elevator.setPosition(Constants.ElevatorConstants.BARGE), maniuplator.setPosition(0, InfeedConstants.BARGE)));
        Operator.povDown().onTrue(new ParallelCommandGroup(elevator.setPosition(Constants.ElevatorConstants.PROCESSOR), maniuplator.setPosition(0, 50)));

        Operator.povLeft().onTrue(maniuplator.setPosition(35,53));
        Operator.povRight().onTrue(maniuplator.setPosition(0, InfeedConstants.CLEARALGEA));

        Operator.leftTrigger(.01).whileTrue(maniuplator.setVelocity(()->Operator.getLeftTriggerAxis()));
        Operator.rightTrigger(.01).whileTrue(maniuplator.setVelocity(()->-Operator.getRightTriggerAxis()));
        Operator.axisGreaterThan(1, -0.06).whileTrue(maniuplator.setRotation(()->Operator.getLeftY()*0.5));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return drivetrain.getAutoCommand(Constants.Trajectorys.onePiece);
    }

    
}

