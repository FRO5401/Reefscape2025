// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

/** Add your docs here. */
public class DrivebaseIOSim implements DrivebaseIO {
    TalonFX leftFalcon = new TalonFX(0);
    TalonFX rightFalcon = new TalonFX(0);

    VoltageOut leftVoltage = new VoltageOut(0);
    VoltageOut rightVoltage = new VoltageOut(0);

    DifferentialDrivetrainSim physicSim = DifferentialDrivetrainSim.createKitbotSim(
        KitbotMotor.kDoubleFalcon500PerSide, 
        KitbotGearing.k8p45, 
        KitbotWheelSize.kSixInch, 
        null // Only needed for noise simulation
        );

    @Override
    public void updateInputs(DrivebaseIOInputs inputs) {
        physicSim.update(0.20);

        var leftSimState = leftFalcon.getSimState();
            leftSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());
        var rightSimState = rightFalcon.getSimState();
            rightSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());

        physicSim.setInputs(leftSimState.getMotorVoltage(), rightSimState.getMotorVoltage());

        // Volts going to drive motors
        inputs.leftOutputVolts = leftSimState.getMotorVoltage();
        inputs.rightOutputVolts = rightSimState.getMotorVoltage();

        //  Speed of drive train sides
        inputs.leftVelocityMetersPerSecond = physicSim.getLeftVelocityMetersPerSecond();
        inputs.rightVelocityMetersPerSecond = physicSim.getRightVelocityMetersPerSecond();

        //  Position on the field for odometry
        inputs.leftPositionMeters = physicSim.getLeftPositionMeters();
        inputs.rightPositionMeters = physicSim.getRightPositionMeters();

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
