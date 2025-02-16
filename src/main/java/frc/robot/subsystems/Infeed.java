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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InfeedConstants;

public class Infeed extends SubsystemBase {

  // Creates holding motors
  SparkMax holdingMotorLeft;
  SparkMax holdingMotorRight;

  // Creates arm motors
  SparkMax rotateMotorLeft;
  SparkMax rotateMotorRight;

  //Creates pivot motor
  // SparkMax pivotMotor;

  //Creates Encoders
  private RelativeEncoder rotateEncoder;
  private RelativeEncoder pivotEncoder;

  //Creates PIDControllers
  private SparkClosedLoopController rotatePID;
  private SparkClosedLoopController pivotPID;

  SparkMaxConfig rotateLeftConfig;
  
  double kP = 0.5;
  double kI = 0.25;
  double kD = 0.0;
  double sped = 0.0;

  
  /** Creates a new Infeed. */
  public Infeed() {
    //Initializes motors
    holdingMotorLeft = new SparkMax(InfeedConstants.HOLDING_MOTOR_LEFT_ID, MotorType.kBrushless);
    holdingMotorRight = new SparkMax(InfeedConstants.HOLDING_MOTOR_RIGHT_ID, MotorType.kBrushless);
    rotateMotorLeft = new SparkMax(InfeedConstants.ROTATE_MOTOR_LEFT_ID, MotorType.kBrushless);
    rotateMotorRight = new SparkMax(InfeedConstants.ROTATE_MOTOR_RIGHT_ID, MotorType.kBrushless);
    // pivotMotor = new SparkMax(InfeedConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

    //Initializes configs
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig holdingLeftConfig = new SparkMaxConfig();
    SparkMaxConfig holdingRightConfig = new SparkMaxConfig();
    rotateLeftConfig = new SparkMaxConfig();
    SparkMaxConfig rotateRightConfig = new SparkMaxConfig();
    SparkMaxConfig pivotConfig = new SparkMaxConfig();

    //Initializes PID controller
    rotatePID = rotateMotorLeft.getClosedLoopController();
    // pivotPID = pivotMotor.getClosedLoopController();

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

    // pivotConfig
    //   .apply(globalConfig);


    // Set the PID configs
    rotateLeftConfig.closedLoop
      .p(kP)
      .i(kI)
      .d(kD);


    // pivotConfig.closedLoop
    //   .p(InfeedConstants.PIVOT_kP)
    //   .i(InfeedConstants.PIVOT_kI)
    //   .d(InfeedConstants.PIVOT_kD);

    //Start position of Rotate Motor
    rotateEncoder.setPosition(InfeedConstants.ROTATE_STOPPED_POSITION);
    // pivotEncoder.setPosition(InfeedConstants.PIVOT_STOPPED_POSITION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Pivot Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Rotate Position", rotateEncoder.getPosition());
    SmartDashboard.putNumber("Rotate PID P", kP);
    SmartDashboard.putNumber("Rotate PID I", kI);
    SmartDashboard.putNumber("Rotate PID D", kD);

    // to set the position using smart dashboard
    double p = SmartDashboard.getNumber("Rotate PID P", 0);
    double i = SmartDashboard.getNumber("Rotate PID I", 0);
    double d = SmartDashboard.getNumber("Rotate PID D", 0);
    if((p != kP)) { rotateLeftConfig.closedLoop.p(p); kP = p; }
    if((i != kI)) { rotateLeftConfig.closedLoop.i(i); kI = i; }
    if((d != kD)) { rotateLeftConfig.closedLoop.d(d); kD = d; }
  }

  public double getRotatePosition(){
    return rotateEncoder.getPosition();
  }

  // public double getPivotPosition(){
  //   return pivotEncoder.getPosition();
  // }

  public double getVelocity(){
    return rotateEncoder.getVelocity();
  }

  public void setVelocity(double velocity){
    holdingMotorLeft.set(velocity);
  }

  public Command setPosition(double rotatePosition, double pivotPosition){
    return runOnce(()->{
      rotatePID.setReference(5, ControlType.kPosition);
      // pivotPID.setReference(pivotPosition, ControlType.kPosition);
    });
  }

}
