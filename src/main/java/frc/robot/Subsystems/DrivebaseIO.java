// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface DrivebaseIO {

    @AutoLog
    public static class DrivebaseIOInputs{
        // Volts going to drive motors
        public double leftOutputVolts = 0.0;
        public double rightOutputVolts = 0.0;

        //  Speed of drive train sides
        public double leftVelocityMetersPerSecond = 0.0;
        public double rightVelocityMetersPerSecond = 0.0;

        //  Position on the field for odometry
        public double leftPositionMeters = 0.0;
        public double rightPositionMeters = 0.0;

        //  Amperage and temperature of motors on drive train
        public double[] leftCurrentAmps = new double[0];
        public double[] leftTempCelsius = new double[0];
        public double[] rightCurrentAmps = new double[0];
        public double[] rightTempCelsius = new double[0];

    }

    public abstract void updateInputs(DrivebaseIOInputs inputs);

    public abstract void setVolts(double left, double right);
} 
