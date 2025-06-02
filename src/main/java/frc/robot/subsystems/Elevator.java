// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Amps;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final TalonFX elevator;
    private Slot0Configs slot0Configs;

    private PositionVoltage PositionPID;

    //Used because PID controllers arent perfect, therefore just saves the ideal value of the pose, which is then passed to change in feed speed;
    private double state = 0;

    private LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
    // the mechanism root node
    private LoggedMechanismRoot2d root = mech.getRoot("root", 2.1,0);

    private LoggedMechanismLigament2d m_elevator = root.append(new LoggedMechanismLigament2d("Elevator", 0.1, 90));

    /** Creates a new Elevator. */
    public Elevator() {
    elevator = new TalonFX(ElevatorConstants.elevatorID);
    elevator.setNeutralMode(NeutralModeValue.Brake);


    slot0Configs = new Slot0Configs();
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(120)).withSupplyCurrentLimit(Amps.of(80)));
    config.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    slot0Configs.kP = Constants.ElevatorConstants.KP; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = Constants.ElevatorConstants.KI; // no output for integrated error
    slot0Configs.kD = Constants.ElevatorConstants.KD; // A velocity of 1 rps results in 0.1 V output
    slot0Configs.kA=Constants.ElevatorConstants.KA;
    slot0Configs.kV=Constants.ElevatorConstants.KV;
    slot0Configs.kG=Constants.ElevatorConstants.KG;
    slot0Configs.GravityType=GravityTypeValue.Elevator_Static;
    

    elevator.getConfigurator().apply(slot0Configs);
    elevator.getConfigurator().apply(config);

    PositionPID = new PositionVoltage(0).withSlot(0);
    elevator.getConfigurator().apply(slot0Configs);
    elevator.setPosition(0);


  }
  public void moveElevator(double speed){
    elevator.set(speed);
  }

      /**
     * returns the inverse of the elevator position as a percent to slow down the robot
     */
  public double getSpeedModifier(){
    return (Math.pow(Math.E, -3*(elevator.getPosition().getValueAsDouble()/ElevatorConstants.SPEED_MODIFIER)));
  }

  public double getElevatorState(){
    Logger.recordOutput("Elevator/Elevator state", state);
    return state;
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Elevator State", mech);

    SmartDashboard.putNumber("Elevator current", elevator.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Stator current", elevator.getStatorCurrent().getValueAsDouble());

    Logger.recordOutput("Elevator/Elevator state", mech);
    // This method will be called once per scheduler run

  }

  public Command setPosition(double pose) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return runOnce(
        () -> {
          elevator.setControl(PositionPID.withPosition(pose));
          state = pose;
          m_elevator.setLength(((Units.inchesToMeters(state*(.75*Math.PI)))/4+.25));
        });
  }

  
}
