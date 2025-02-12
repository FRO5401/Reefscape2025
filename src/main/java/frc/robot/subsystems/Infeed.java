// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InfeedConstants;

public class Infeed extends SubsystemBase {

  // Creates holding motors
  SparkMax holdingMotorLeft;
  SparkMax holdingMotorRight;

  // Creates arm motors
  SparkMax rotateMotorLeft;
  SparkMax rotateMotorRight;
  //Creates Encoders
  private RelativeEncoder rotateEncoder;

  //Creates PIDController
  private SparkClosedLoopController rotatePID;

  
  /** Creates a new Infeed. */
  public Infeed() {
    //Initializes motors
    holdingMotorLeft = new SparkMax(InfeedConstants.HOLDING_MOTOR_LEFT_ID, MotorType.kBrushless);
    holdingMotorRight = new SparkMax(InfeedConstants.HOLDING_MOTOR_RIGHT_ID, MotorType.kBrushless);
    rotateMotorLeft = new SparkMax(InfeedConstants.ARM_MOTOR_LEFT_ID, MotorType.kBrushless);
    rotateMotorRight = new SparkMax(InfeedConstants.ARM_MOTOR_RIGHT_ID, MotorType.kBrushless);

    //Initializes configs
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig holdingLeftConfig = new SparkMaxConfig();
    SparkMaxConfig holdingRightConfig = new SparkMaxConfig();
    SparkMaxConfig rotateLeftConfig = new SparkMaxConfig();
    SparkMaxConfig rotateRightConfig = new SparkMaxConfig();

    //Initializes PID controller
    rotatePID = rotateMotorLeft.getClosedLoopController();
    
    //Gets Encoder for Rotate Motor
    rotateEncoder = rotateMotorLeft.getEncoder();

    // Sets motor configs
    globalConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);

    holdingLeftConfig
      .apply(globalConfig)
      .inverted(false);
    
    holdingRightConfig
      .apply(globalConfig)
      .inverted(true)
      .follow(holdingMotorLeft);

    rotateLeftConfig
      .apply(globalConfig)
      .inverted(false);
    
    rotateRightConfig
      .apply(globalConfig)
      .inverted(true)
      .follow(rotateMotorLeft);

    // Set the PID config
    rotateLeftConfig.closedLoop
      .p(InfeedConstants.ARM_kP)
      .i(InfeedConstants.ARM_kI)
      .d(InfeedConstants.ARM_kD);
    
    //Start position of Rotate Motor
    rotateEncoder.setPosition(InfeedConstants.STOPPED_POSITION);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getPosition(){
    return rotateEncoder.getPosition();
  }

  public double getVelocity(){
    return rotateEncoder.getVelocity();
  }

  public void setInfeed(int position, double velocity){
    rotatePID.setReference(position, ControlType.kPosition);
    holdingMotorLeft.set(velocity);
  }
}
