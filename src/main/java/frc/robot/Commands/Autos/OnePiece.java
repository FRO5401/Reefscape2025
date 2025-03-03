// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.InfeedConstants.IntakeConstants;
import frc.robot.Constants.InfeedConstants.PivotConstants;
import frc.robot.Constants.Trajectorys;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePiece extends SequentialCommandGroup {
  /** Creates a new OnePiece. */
  public OnePiece(CommandSwerveDrivetrain drivetrain, Elevator elevator, Manipulator manipulator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(elevator.setPosition(ElevatorConstants.PROCESSOR),manipulator.setPosition(IntakeConstants.HOLD_CORAL, PivotConstants.BARGE)),
      drivetrain.getAutoCommand(Trajectorys.onePiece),
      new ParallelCommandGroup(elevator.setPosition(ElevatorConstants.L2), manipulator.setPosition(IntakeConstants.HOLD_CORAL, PivotConstants.L4)),
      Commands.waitSeconds(1.5),
      drivetrain.getAutoCommand(Trajectorys.onePieceReef),
      Commands.waitSeconds(3.5),
      manipulator.setVelocity(()->-1),
      Commands.waitSeconds(1),
      manipulator.setVelocity(()->0),
      drivetrain.getAutoCommand(Trajectorys.backup)
    );
  }
}
