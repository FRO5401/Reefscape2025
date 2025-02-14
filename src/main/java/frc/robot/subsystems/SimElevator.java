// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSimConstants;

public class SimElevator extends SubsystemBase {
  //Gearbox for elevator
  DCMotor elevatorKraken;
  //PID controller for sim
  ProfiledPIDController elevatorPID;

  ElevatorFeedforward elevatorControl;

  ElevatorSim elevatorSim;

  Encoder encoder;
  EncoderSim encoderSim;

  Mechanism2d mech2d;
  MechanismRoot2d elevatorbase;
  MechanismLigament2d elevatorstage1;
  MechanismLigament2d elevatorstage2;
  MechanismLigament2d elevatorstage3;

  /** Creates a new SimElevator. */
  public SimElevator() {
    elevatorKraken = DCMotor.getKrakenX60(ElevatorConstants.elevatorID);

    elevatorPID = new ProfiledPIDController(
      ElevatorConstants.KP, 
      ElevatorConstants.KI, 
      ElevatorConstants.KD, 
      new TrapezoidProfile.Constraints(2.45, 2.45));
    
    elevatorControl = new ElevatorFeedforward(
      ElevatorSimConstants.kElevatorkS, 
      ElevatorConstants.KG, 
      ElevatorConstants.KV,
      ElevatorConstants.KA);

    encoder = new Encoder(
      ElevatorSimConstants.kEncoderAChannel, 
      ElevatorSimConstants.kEncoderBChannel);
    encoder.setDistancePerPulse(0);
    encoderSim = new EncoderSim(encoder);
    
    elevatorSim = new ElevatorSim(
      elevatorKraken, 
      ElevatorSimConstants.ElevatorGearing, 
      ElevatorSimConstants.kCarriageMass, 
      ElevatorSimConstants.kElevatorDrumRadius,
      ElevatorSimConstants.kMinElevatorHeightMeters, 
      ElevatorSimConstants.kMaxElevatorHeightMeters, 
      true, 
      ElevatorSimConstants.kMinElevatorHeightMeters,  
      null
      );
    
    mech2d = new Mechanism2d(25, ElevatorSimConstants.kMinElevatorHeightMeters);
    elevatorbase = mech2d.getRoot(
      "Elevator Root", 
      ElevatorSimConstants.kMinElevatorHeightMeters, 
      0);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
