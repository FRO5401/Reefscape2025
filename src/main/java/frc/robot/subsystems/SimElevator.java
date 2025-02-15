// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.ElevatorConstants.ElevatorSimConstants;

public class SimElevator extends SubsystemBase {
  //Gearbox for elevator
  DCMotor elevatorKraken;
  TalonFX kraken;
  TalonFXSimState simKraken;
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
    kraken = new TalonFX(ElevatorConstants.elevatorID);
    simKraken = kraken.getSimState();
    
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
      ElevatorSimConstants.kEncoderBChannel
    );
    encoder.setDistancePerPulse(ElevatorSimConstants.kElevatorEncoderDistPerPulse);
    encoderSim = new EncoderSim(encoder);
    
    elevatorSim = new ElevatorSim(
      elevatorKraken, 
      ElevatorSimConstants.ElevatorGearing, 
      ElevatorSimConstants.kCarriageMass, 
      ElevatorSimConstants.kElevatorDrumRadius,
      ElevatorSimConstants.kMinElevatorHeightMeters, 
      ElevatorSimConstants.kMaxElevatorHeightMeters, 
      true, 
      ElevatorSimConstants.kMinElevatorHeightMeters  
      
      );
    
    mech2d = new Mechanism2d(Swerve.robotWidth, Swerve.robotLength);
    elevatorbase = mech2d.getRoot(
      "Elevator Root", 10, 0
      );

      elevatorstage1 =
      elevatorbase.append(
          new MechanismLigament2d("Stage1", elevatorSim.getPositionMeters(), 90, 6, new Color8Bit(Color.kAntiqueWhite)));
      elevatorstage2 = elevatorstage1.append(
        new MechanismLigament2d("Stage2", elevatorSim.getPositionMeters(), 0, 6, new Color8Bit(Color.kBeige)));
      elevatorstage3 = elevatorstage2.append(
        new MechanismLigament2d("Stage3", elevatorSim.getPositionMeters(), 0, 6, new Color8Bit(Color.kGreen)));

      SmartDashboard.putData("Elevator Sim", mech2d);
  }
  @Override
  public void simulationPeriodic(){
    elevatorSim.setInput(simKraken.getMotorVoltage() * RobotController.getBatteryVoltage());
    elevatorSim.update(0.020);
    encoderSim.setDistance(elevatorSim.getPositionMeters());
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps())
    );
  
  }

  public Command reachGoal(double goal){
    

    return runOnce(() -> {
      elevatorPID.setGoal(goal);

      double pidOutput = elevatorPID.calculate(encoder.getDistance());
      double feedForwardOutput = elevatorControl.calculate(elevatorPID.getSetpoint().velocity);
      simKraken.setSupplyVoltage(pidOutput + feedForwardOutput);
  });
  }

  public Command stop(){
    return runOnce(() -> {
    elevatorPID.setGoal(0.0);
    simKraken.setSupplyVoltage(0);
    });
  }

  public void updateTelemetry(){
    elevatorstage1.setLength(encoder.getDistance());
    elevatorstage2.setLength(encoder.getDistance());
    elevatorstage3.setLength(encoder.getDistance());
  }

  public void close(){

    encoder.close();
    kraken.close();
    mech2d.close();

  }

  @Override
  public void periodic() {
    updateTelemetry();
    // This method will be called once per scheduler run
  }
}
