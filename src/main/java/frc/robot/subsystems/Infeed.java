// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.InfeedConstants;

public class Infeed extends SubsystemBase {

  // Creates motors
  SparkMax holdingMotorLeft;
  SparkMax holdingMotorRight;
  SparkMax rotateMotorLeft;
  SparkMax rotateMotorRight;
  // SparkMax pivotMotor;

  //Creates Encoders
  private RelativeEncoder rotateLeftEncoder;
  private RelativeEncoder rotateRightEncoder;
  private RelativeEncoder pivotEncoder;

  //Creates PIDControllers
  private SparkClosedLoopController rotateLeftPID;
  private SparkClosedLoopController rotateRightPID;
  private SparkClosedLoopController pivotPID;

  //Creates configs
  SparkMaxConfig globalConfig;
  SparkMaxConfig holdingLeftConfig;
  SparkMaxConfig holdingRightConfig;
  SparkMaxConfig rotateLeftConfig;
  SparkMaxConfig rotateRightConfig;
  SparkMaxConfig pivotConfig;

  //Create Beam Break
  DigitalInput beamBreak;

  
  /** Creates a new Infeed. */
  public Infeed() {
    //Initializes motors
    holdingMotorLeft = new SparkMax(InfeedConstants.HOLDING_MOTOR_LEFT_ID, MotorType.kBrushless);
    holdingMotorRight = new SparkMax(InfeedConstants.HOLDING_MOTOR_RIGHT_ID, MotorType.kBrushless);
    rotateMotorLeft = new SparkMax(InfeedConstants.ROTATE_MOTOR_LEFT_ID, MotorType.kBrushless);
    rotateMotorRight = new SparkMax(InfeedConstants.ROTATE_MOTOR_RIGHT_ID, MotorType.kBrushless);
    // pivotMotor = new SparkMax(InfeedConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

    

    //Initializes configs
    globalConfig = new SparkMaxConfig();
    holdingLeftConfig = new SparkMaxConfig();
    holdingRightConfig = new SparkMaxConfig();
    rotateLeftConfig = new SparkMaxConfig();
    rotateRightConfig = new SparkMaxConfig();
    pivotConfig = new SparkMaxConfig();

    //Initializes PID controllers
    rotateLeftPID = rotateMotorLeft.getClosedLoopController();
    rotateRightPID = rotateMotorRight.getClosedLoopController();
    // pivotPID = pivotMotor.getClosedLoopController();

    //Initalize BeamBreak
    beamBreak = new DigitalInput(2);

    //Gets Encoder for Rotate Motors
    rotateLeftEncoder = rotateMotorLeft.getEncoder();
    rotateRightEncoder = rotateMotorRight.getEncoder();

    // Sets motor configs
    globalConfig
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);

    holdingLeftConfig
      .apply(globalConfig)
      .inverted(false);
    
    holdingRightConfig
      .apply(globalConfig)
      .inverted(true);

    rotateLeftConfig
      .apply(globalConfig)
      .inverted(false);
    
    rotateRightConfig
      .apply(globalConfig)
      .inverted(false);

    // pivotConfig
    //   .apply(globalConfig);


    // Set the PID configs
    rotateLeftConfig.closedLoop
      .p(Constants.InfeedConstants.ROTATE_KP)
      .i(Constants.InfeedConstants.ROTATE_kI)
      .d(Constants.InfeedConstants.ROTATE_kD);

    rotateRightConfig.closedLoop
    .p(Constants.InfeedConstants.ROTATE_KP)
    .i(Constants.InfeedConstants.ROTATE_kI)
    .d(Constants.InfeedConstants.ROTATE_kD);

    // pivotConfig.closedLoop
    //   .p(InfeedConstants.PIVOT_kP)
    //   .i(InfeedConstants.PIVOT_kI)
    //   .d(InfeedConstants.PIVOT_kD);


    holdingMotorLeft.configure(holdingLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    holdingMotorRight.configure(holdingRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotateMotorLeft.configure(rotateLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotateMotorRight.configure(rotateRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    //Start position of encoders
    rotateLeftEncoder.setPosition(InfeedConstants.ROTATE_STOPPED_POSITION);
    rotateRightEncoder.setPosition(InfeedConstants.ROTATE_STOPPED_POSITION);
    // pivotEncoder.setPosition(InfeedConstants.PIVOT_STOPPED_POSITION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Pivot Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Rotate Left Position", rotateLeftEncoder.getPosition());
    SmartDashboard.putNumber("Rotate Right Position", rotateRightEncoder.getPosition());
    SmartDashboard.putBoolean("BeamBreak", getBeamBreak());

  }

  public double getRotateLeftPosition(){
    return rotateLeftEncoder.getPosition();
  }

  public double getRotateRightPosition(){
    return rotateRightEncoder.getPosition();
  }

  public boolean getBeamBreak(){
    return beamBreak.get();
  }

  // public double getPivotPosition(){
  //   return pivotEncoder.getPosition();
  // }

  public double getLeftVelocity(){
    return rotateLeftEncoder.getVelocity();
  }

  public double getRightVelocity(){
    return rotateRightEncoder.getVelocity();
  }

  public Command setVelocity(double velocity){
    return runOnce(()->{
      holdingMotorLeft.set(velocity);
      holdingMotorRight.set(velocity);
    });
  }

  public Command setPosition(double rotatePosition, double pivotPosition){
    return runOnce(()->{
      rotateLeftPID.setReference(0.5, ControlType.kPosition);
      rotateRightPID.setReference(0.5, ControlType.kPosition);
      // pivotPID.setReference(pivotPosition, ControlType.kPosition);
    });
  }

  public Command setState(double rotatePosition, double pivotPosition, double velocity){
    return runOnce(()->{
      new ParallelCommandGroup(
        setPosition(rotatePosition, pivotPosition),
        setVelocity(velocity)
      );
    });
  }
}