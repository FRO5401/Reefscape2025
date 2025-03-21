// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InfeedConstants;


public class Manipulator extends SubsystemBase {
  SparkMax intakeLeft = new SparkMax(InfeedConstants.IntakeConstants.INTAKE_MOTOR_LEFT, MotorType.kBrushless);
  SparkMax intakeRight = new SparkMax(InfeedConstants.IntakeConstants.INTAKE_MOTOR_RIGHT, MotorType.kBrushless);

  SparkMax rotateLeft = new SparkMax(InfeedConstants.IntakeConstants.ROTATE_MOTOR_LEFT, MotorType.kBrushless);
  SparkMax rotateRight = new SparkMax(InfeedConstants.IntakeConstants.ROTATE_MOTOR_RIGHT, MotorType.kBrushless);

  SparkMax pivot = new SparkMax(InfeedConstants.PivotConstants.PIVOT_ID, MotorType.kBrushless);


  

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
      .follow(intakeLeft, true);

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
      .smartCurrentLimit(70)
      .idleMode(IdleMode.kBrake)
      .encoder
        .positionConversionFactor(45);
      
      
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
      .iZone(.2)
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

 public Command setVelocity(DoubleSupplier velocity){
  return runOnce(()->{
    intakeLeft.set(velocity.getAsDouble());
  });
  
 }

 public Command stopIntake(){
  return run(()->{
    intakeLeft.set(0);
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

 public BooleanSupplier iscurrentspiked(){
  return ()->leftIntakeCurrent()>40;
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Position", pivotEncoeer.getPosition());
    SmartDashboard.putNumber("Pinch Position", rotateLeftEncoder.getPosition());
    SmartDashboard.putNumber("Pinch right Position", rotateRightEncoder.getPosition());

    SmartDashboard.putBoolean("HasCoral", getBeamBreak());
    SmartDashboard.putNumber("right Intake Current", rightIntakeCurrent());
    SmartDashboard.putNumber("Left Intake Current", leftIntakeCurrent());


  }
}
