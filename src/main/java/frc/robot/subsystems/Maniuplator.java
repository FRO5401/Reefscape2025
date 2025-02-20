// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.InfeedConstants;

public class Maniuplator extends SubsystemBase {
  SparkMax intakeLeft = new SparkMax(Constants.InfeedConstants.INTAKE_MOTOR_LEFT, MotorType.kBrushless);
  SparkMax intakeRight = new SparkMax(Constants.InfeedConstants.INTAKE_MOTOR_RIGHT, MotorType.kBrushless);

  SparkMax rotateLeft = new SparkMax(Constants.InfeedConstants.ROTATE_MOTOR_LEFT, MotorType.kBrushless);
  SparkMax rotateRight = new SparkMax(Constants.InfeedConstants.ROTATE_MOTOR_RIGHT, MotorType.kBrushless);

  SparkMax pivot = new SparkMax(Constants.InfeedConstants.PIVOT_ID, MotorType.kBrushless);

  RelativeEncoder rotateEncoder = rotateLeft.getEncoder();

  RelativeEncoder pivotEncoeer = pivot.getEncoder();

  SparkClosedLoopController rotatePID = rotateLeft.getClosedLoopController();

  SparkClosedLoopController PivotPID = pivot.getClosedLoopController();
  
  
  SparkMaxConfig globalConfig;
  SparkMaxConfig intakeLeftConfig;
  SparkMaxConfig intakeRightConfig;
  SparkMaxConfig rotateLeftConfig;
  SparkMaxConfig rotateRightConfig;
  SparkMaxConfig pivotConfig;

  DigitalInput beamBreak;

  /** Creates a new Maniuplator. */
  public Maniuplator() {

    globalConfig = new SparkMaxConfig();
    intakeLeftConfig = new SparkMaxConfig();
    intakeRightConfig = new SparkMaxConfig();
    rotateLeftConfig = new SparkMaxConfig();
    rotateRightConfig = new SparkMaxConfig();
    pivotConfig = new SparkMaxConfig();

      globalConfig
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kCoast);

    intakeLeftConfig
      .apply(globalConfig)
      .inverted(false);
    
    intakeRightConfig
      .apply(globalConfig)
      .follow(intakeLeft, true);

    rotateLeftConfig
      .apply(globalConfig)
      .inverted(false)
      .encoder
        .positionConversionFactor(50);
      
    
    rotateRightConfig
      .apply(globalConfig);
      //.follow(rotateLeft, true);
      

    pivotConfig
      .apply(globalConfig)
      .inverted(true)
      .encoder
        .positionConversionFactor(15);
      
    rotateLeftConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(InfeedConstants.ROTATE_KP,InfeedConstants.ROTATE_kI,InfeedConstants.ROTATE_kD);
      //.pidf(InfeedConstants.ROTATE_KP,InfeedConstants.ROTATE_kI,InfeedConstants.ROTATE_kD,InfeedConstants.ROTATE_KF);
      
      

    pivotConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(Constants.InfeedConstants.PIVOT_KP)
      .i(Constants.InfeedConstants.PIVOT_KI)
  
      .d(Constants.InfeedConstants.PIVOT_KD);


      
    intakeLeft.configure(intakeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeRight.configure(intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotateLeft.configure(rotateLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotateRight.configure(rotateRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    //Start position of encoders
    rotateEncoder.setPosition(0);
    pivotEncoeer.setPosition(0);
  }

 public Command setPosition(double rotatePosition, double pivotPosition){
  return runOnce(()->{
    rotatePID.setReference(rotatePosition, ControlType.kPosition);
    PivotPID.setReference(pivotPosition, ControlType.kPosition);
  });


 }

 public Command setVelocity(DoubleSupplier velocity){
  return run(()->{
    intakeLeft.set(velocity.getAsDouble());
  }).finallyDo(()->intakeLeft.set(0));
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("rotate Value", rotateLeft.getAppliedOutput());    
    SmartDashboard.putNumber("rotate Position", rotateEncoder.getPosition());
    SmartDashboard.putNumber("pivot Position", pivotEncoeer.getPosition());

  }
}
