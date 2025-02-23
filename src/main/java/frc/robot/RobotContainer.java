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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModeConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSimConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorSimulation;
import frc.robot.subsystems.drive.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.drive.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private SwerveDriveSimulation driveSimulation = null;
  // Subsystems
  private final Drive drive;
  // Controller
  private final CommandXboxController driver = Controls.driver;
  private final CommandXboxController operator = Controls.operator;
  
  private final Encoder elevatorEncoder = new Encoder(ElevatorSimConstants.kEncoderAChannel, ElevatorSimConstants.kEncoderBChannel);
    private final PWMSparkMax elevatorMotor = new PWMSparkMax(ElevatorConstants.elevatorID);

    private final Elevator m_elevator = new Elevator(elevatorEncoder, elevatorMotor);

    private ElevatorSimulation m_elevatorSimulation;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (ModeConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
                        .withGyro(GyroSimulation.getPigeon2())
                        .withSwerveModule(() -> new SwerveModuleSimulation(
                                DCMotor.getKrakenX60(1),
                                DCMotor.getFalcon500(1),
                                TunerConstants.FrontLeft.DriveMotorGearRatio,
                                TunerConstants.FrontLeft.SteerMotorGearRatio,
                                Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                                Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                                Inches.of(2),
                                KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                                1.2));
                driveSimulation = new SwerveDriveSimulation(config, new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]));
                break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }
    CommandScheduler.getInstance().setDefaultCommand(m_elevator, m_elevator.depower());

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY() * ModeConstants.flipped(),
            () -> -driver.getLeftX() * ModeConstants.flipped(),
            () -> -driver.getRawAxis(5) * ModeConstants.flipped()));

    // Lock to 0° when A button is held
    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driver.getLeftY() * ModeConstants.flipped(),
                () -> -driver.getLeftX() * ModeConstants.flipped(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    
    new Trigger(()->{return RobotState.isEnabled() && RobotState.isTeleop();}).onTrue(m_elevator.initConstants());

    operator.a().onTrue(m_elevator.goToHeight(0.25));
    operator.b().onTrue(m_elevator.goToHeight(0.50));
    operator.y().onTrue(m_elevator.goToHeight(0.75));
    operator.x().onTrue(m_elevator.goToHeight(1.00));
    operator.povDown().onTrue(m_elevator.goToHeight(0.00));
        
    operator.start().whileTrue(m_elevator.manualControl(()->-0.2*operator.getLeftY()));
    operator.back().whileTrue(m_elevator.closedLoopManualSetpoint(()->-1.25*operator.getLeftY()));

  }
  public void simulationPeriodic()
  {
      m_elevatorSimulation.elevatorSimulationPeriodic();
  }

  public void close() 
    {
        elevatorEncoder.close();
        elevatorMotor.close();
        m_elevator.close();
    }

  public Command getAutonomousCommand() {
    return null;
  }
  public void resetSimulation() {
        if (ModeConstants.currentMode != ModeConstants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void displaySimFieldToAdvantageScope() {
        if (ModeConstants.currentMode != ModeConstants.Mode.SIM) return;

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Notes",
                SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(new Pose3d[0]));
    }
}
