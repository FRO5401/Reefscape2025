// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.InfeedConstants.IntakeConstants;
import frc.robot.Constants.InfeedConstants.PivotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePiece extends SequentialCommandGroup {
  /** Creates a new OnePiece. */
  public OnePiece(CommandSwerveDrivetrain drivetrain, Elevator elevator, Manipulator manipulator) {
    Trigger hasAlgea = new Trigger(manipulator.iscurrentspiked()).debounce(.1);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
      new ParallelCommandGroup(elevator.setPosition(ElevatorConstants.PROCESSOR),manipulator.setPosition(IntakeConstants.HOLD_CORAL, PivotConstants.BARGE)),
      RobotContainer.alignAndDriveToReef(21,Units.inchesToMeters(-2), VisionConstants.AUTO_DISTANCE),
      new ParallelCommandGroup(elevator.setPosition(ElevatorConstants.L4), manipulator.setPosition(IntakeConstants.HOLD_CORAL, PivotConstants.L4)),
      RobotContainer.alignAndDriveToReef(21,Units.inchesToMeters(-2), VisionConstants.REEF_DISTANCE),

      Commands.waitSeconds(1),
      manipulator.setClaw(IntakeConstants.HOLD_ALGEA),
      manipulator.setVelocity(()->-1),
      Commands.waitSeconds(.5),
      manipulator.setVelocity(()->0),

      RobotContainer.alignAndDriveToReef(21,Units.inchesToMeters(-2.3), VisionConstants.AUTO_DISTANCE),
      new ParallelCommandGroup(
                        manipulator.setPosition(
                                IntakeConstants.HOLD_ALGEA,
                                PivotConstants.STRAIGHTOUT),
                        elevator.setPosition(ElevatorConstants.L2 - 7)),
      Commands.waitSeconds(1),
      manipulator.setVelocity(()->1),
      RobotContainer.alignAndDriveToReef(21,Units.inchesToMeters(0), VisionConstants.ALGEA_DISTANCE).until(hasAlgea),
      RobotContainer.alignAndDriveToReef(21,Units.inchesToMeters(0), VisionConstants.AUTO_DISTANCE),

      Commands.waitSeconds(2),
      RobotContainer.alignAndDriveToReef(14,Units.inchesToMeters(0), VisionConstants.BARGE_DISTANCE),
      Commands.waitSeconds(1),
      new ParallelCommandGroup(
        elevator.setPosition(ElevatorConstants.BARGE),
        manipulator.setPosition(
                IntakeConstants.HOLD_ALGEA,
                PivotConstants.BARGE)
      ),

      Commands.waitSeconds(5),
      manipulator.setVelocity(()->-1)
    );         
  }
}
