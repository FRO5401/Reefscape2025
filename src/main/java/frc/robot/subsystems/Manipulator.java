// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.InfeedConstants.IntakeConstants;


public class Manipulator extends SubsystemBase {
  SparkMax intakeLeft = new SparkMax(InfeedConstants.IntakeConstants.INTAKE_MOTOR_LEFT, MotorType.kBrushless);
  SparkMax intakeRight = new SparkMax(InfeedConstants.IntakeConstants.INTAKE_MOTOR_RIGHT, MotorType.kBrushless);

  SparkMax rotateLeft = new SparkMax(InfeedConstants.IntakeConstants.ROTATE_MOTOR_LEFT, MotorType.kBrushless);
  SparkMax rotateRight = new SparkMax(InfeedConstants.IntakeConstants.ROTATE_MOTOR_RIGHT, MotorType.kBrushless);

  SparkMax pivot = new SparkMax(InfeedConstants.PivotConstants.PIVOT_ID, MotorType.kBrushless);

  //.02 is the schedular cycle time, 
  Debouncer currentFilter = new Debouncer(.1, DebounceType.kBoth);


  

  RelativeEncoder rotateLeftEncoder = rotateLeft.getEncoder();
  RelativeEncoder rotateRightEncoder = rotateRight.getEncoder();


  RelativeEncoder pivotEncoeer = pivot.getEncoder();

  SparkClosedLoopController rotateLeftPID = rotateLeft.getClosedLoopController();
  SparkClosedLoopController rotateRightPID = rotateRight.getClosedLoopController();


  SparkClosedLoopController PivotPID = pivot.getClosedLoopController();
  
  
  SparkMaxConfig globalConfig;
  SparkMaxConfig intakeLeftConfig;
  SparkMaxConfig intakeRightConfig;
  SparkMaxConfig rotateLeftConfig;
  SparkMaxConfig rotateRightConfig;
  SparkMaxConfig pivotConfig;

  DigitalInput beamBreak;

  /** Creates a new Maniuplator. */
  public Manipulator() {

    

    globalConfig = new SparkMaxConfig();
    intakeLeftConfig = new SparkMaxConfig();
    intakeRightConfig = new SparkMaxConfig();
    rotateLeftConfig = new SparkMaxConfig();
    rotateRightConfig = new SparkMaxConfig();
    pivotConfig = new SparkMaxConfig();

    beamBreak = new DigitalInput(InfeedConstants.BEAM_BREAK_ID);

      globalConfig
      .smartCurrentLimit(20)
      .idleMode(IdleMode.kBrake)
      .disableFollowerMode();

    intakeLeftConfig
      .apply(globalConfig)
      .inverted(false);
    
    intakeRightConfig
      .apply(globalConfig)
      .inverted(true);

    rotateLeftConfig
      .apply(globalConfig)
      .inverted(false)
      .disableFollowerMode()
      .encoder
        .positionConversionFactor(16);
      
    
    rotateRightConfig
    .apply(globalConfig)
    .inverted(false)
    .disableFollowerMode()
    .encoder
      .positionConversionFactor(16);
      

    pivotConfig
      .apply(globalConfig)
      .inverted(true)
      .smartCurrentLimit(80)
      .idleMode(IdleMode.kBrake)
      .encoder
        .positionConversionFactor(13.8);
      
      
    rotateLeftConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(InfeedConstants.IntakeConstants.ROTATE_KP,
        InfeedConstants.IntakeConstants.ROTATE_kI,
        InfeedConstants.IntakeConstants.ROTATE_kD);

      rotateRightConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(InfeedConstants.IntakeConstants.ROTATE_KP,
      InfeedConstants.IntakeConstants.ROTATE_kI,
        InfeedConstants.IntakeConstants.ROTATE_kD);
      
      

    pivotConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .iZone(5)
      .p(InfeedConstants.PivotConstants.PIVOT_KP)
      .i(InfeedConstants.PivotConstants.PIVOT_KI)
      .d(InfeedConstants.PivotConstants.PIVOT_KD);


      
    intakeLeft.configure(intakeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeRight.configure(intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotateLeft.configure(rotateLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotateRight.configure(rotateRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    //Start position of encoders
    rotateLeftEncoder.setPosition(0);
    rotateRightEncoder.setPosition(0);

    pivotEncoeer.setPosition(0);
  }
 public Command zeroPinch(){
  return runOnce(()->{
    rotateLeftEncoder.setPosition(0);
    pivotEncoeer.setPosition(0);
  });
 }

 public Command setPosition(double rotatePosition, double pivotPosition){
  return runOnce(()->{
    rotateLeftPID.setReference(rotatePosition, ControlType.kPosition);
    rotateRightPID.setReference(-rotatePosition, ControlType.kPosition);
    PivotPID.setReference(pivotPosition, ControlType.kPosition);
  });
 }

 public Command setClaw(double rotatePosition){
  return runOnce(()->{
    rotateLeftPID.setReference(rotatePosition, ControlType.kPosition);
    rotateRightPID.setReference(-rotatePosition, ControlType.kPosition);
  });
}


//overloaded to make it so that we can add backspin to the algea if needed
 public Command setVelocity(double leftVelocity, double rightVelocity){
  return runOnce(()->{
    intakeLeft.set(leftVelocity);
    intakeRight.set(rightVelocity);
  });
 }

 public Command setVelocity(DoubleSupplier velocity){
  return runOnce(()->{
    intakeLeft.set(velocity.getAsDouble());
    intakeRight.set(velocity.getAsDouble());
  });
 }

     //selects the command based off of elevator pose
public Command expelCommand(Elevator elevator){
      return new SelectCommand<>(
          // Maps elevator state to different manipulator speeds
          Map.ofEntries(
              Map.entry(ElevatorConstants.BARGE, setVelocity(IntakeConstants.TELEOP_REPEL_ALGEA, 0)),
              Map.entry(ElevatorConstants.L4, setVelocity(()->IntakeConstants.TELEOP_REPEL_CORAL)),
              Map.entry(ElevatorConstants.L3, setVelocity(()->IntakeConstants.TELEOP_REPEL_CORAL)),
              Map.entry(ElevatorConstants.L2, setVelocity(()->IntakeConstants.TELEOP_REPEL_CORAL)),
              Map.entry(ElevatorConstants.PROCESSOR, setVelocity(()->IntakeConstants.TELEOP_REPEL_ALGEA))),
          elevator::getElevatorState);
}



 public Command stopIntake(){
  return run(()->{
    intakeLeft.set(0);
    intakeRight.set(0);
  });
 }

 public Command setRotation(DoubleSupplier position){
  return runOnce(()->{
    rotateLeft.set(position.getAsDouble());
    rotateRight.set(-position.getAsDouble());
  });
 }

 public boolean getBeamBreak(){
  return !beamBreak.get();
 }

 public double rightIntakeCurrent(){
  return intakeRight.getOutputCurrent();
 }

 public double leftIntakeCurrent(){
  return intakeLeft.getOutputCurrent();
 }

 public BooleanSupplier isCurrentSpiked(){
  return ()->currentFilter.calculate((leftIntakeCurrent()>21));
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pinch Position", rotateLeftEncoder.getPosition());
    SmartDashboard.putNumber("Pinch right Position", rotateRightEncoder.getPosition());

    SmartDashboard.putBoolean("HasCoral", getBeamBreak());

    SmartDashboard.putBoolean("HasAlgea", isCurrentSpiked().getAsBoolean());
    SmartDashboard.putNumber("right Intake Current", rightIntakeCurrent());
    SmartDashboard.putNumber("Left Intake Current", leftIntakeCurrent());


  }
}
