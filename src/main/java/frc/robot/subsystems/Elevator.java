// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private TalonFX elevator;
  Slot0Configs slot0Configs;
  double kS;
  double kP;
  double kV;
  double kD;
  double kI;

  /** Creates a new Elevator. */
  public Elevator() {
    elevator = new TalonFX(ElevatorConstants.elevatorID);
    elevator.setNeutralMode(NeutralModeValue.Brake);


    kS = 0.105401;
    kV = 0.105401;
    kP = 0.25401;
    kI = 0.05401;
    kD = 0.0015401;
    /* 01!!!! */


    slot0Configs = new Slot0Configs();
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;

    elevator.getConfigurator().apply(slot0Configs);


    SmartDashboard.putNumber("S Gain", kS);
    SmartDashboard.putNumber("V Gain", kV);
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
  }
  public void moveElevator(double speed){
    elevator.set(speed);
  }

  @Override
  public void periodic() {
    double s = SmartDashboard.getNumber("S Gain", 0);
    double v = SmartDashboard.getNumber("V Gain", 0);
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    
    if((s != kS)) { slot0Configs.kS = s; }
    if((v != kV)) { slot0Configs.kV = v; }
    if((p != kP)) { slot0Configs.kP = p; }
    if((i != kI)) { slot0Configs.kI = i; }
    if((d != kD)) { slot0Configs.kD = d; }

    elevator.getConfigurator().apply(slot0Configs);
    // This method will be called once per scheduler run
  }
  public void setPosition(double Position){
    elevator.setPosition(Position);
  }
}
