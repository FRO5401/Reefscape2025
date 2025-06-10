// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Elevator extends SubsystemBase {
    /** Creates a new Elevator. */
    public Elevator() {}

    public abstract Command setPosition(double position);

    public abstract double getPosition();

    @Override
    public void periodic() {
        Logger.recordOutput("Elevator/ElevatorPosition", getPosition());
    }

    /**
     * returns the inverse of the elevator position as a percent to slow down the robot
    */
    public double getSpeedModifier(){
        return (Math.pow(Math.E, -3*(getPosition()/ElevatorConstants.SPEED_MODIFIER)));
    }

}
