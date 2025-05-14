// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class DrivebaseIOSim implements DrivebaseIO {
    TalonFX leftFalcon = new TalonFX(0);
    TalonFX rightFalcon = new TalonFX(0);

    VoltageOut leftVoltage = new VoltageOut(0);
    VoltageOut rightVoltage = new VoltageOut(0);

    @Override
    public void updateInputs(DrivebaseIOInputs inputs) {

        var leftSimState = leftFalcon.getSimState();
        var rightSimState = rightFalcon.getSimState();

        // Volts going to drive motors
        inputs.leftOutputVolts = leftSimState.getMotorVoltage();
        inputs.rightOutputVolts = rightSimState.getMotorVoltage();

        //  Speed of drive train sides
        inputs.leftVelocityMetersPerSecond = 0.0;
        inputs.rightVelocityMetersPerSecond = 0.0;

        //  Position on the field for odometry
        inputs.leftPositionMeters = 0.0;
        inputs.rightPositionMeters = 0.0;

        //  Amperage and temperature of motors on drive train
        inputs.leftCurrentAmps = new double[] {leftSimState.getTorqueCurrent()};
        inputs.leftTempCelsius = new double[0];
        inputs.rightCurrentAmps = new double[]{rightSimState.getTorqueCurrent()};
        inputs.rightTempCelsius = new double[0];
    }

    @Override
    public void setVolts(double left, double right) {
        leftFalcon.setControl(leftVoltage.withOutput(left));
        rightFalcon.setControl(rightVoltage.withOutput(right));

    }}
