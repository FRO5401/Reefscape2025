// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  SparkMax climberLeft;
  SparkMax climberRight;
  SparkMaxConfig climberGlobal;
  SparkMaxConfig climberLeftConfig;
  SparkMaxConfig climberRightConfig;
  RelativeEncoder climberEncoder;
  DutyCycleEncoder REVEncoder;

  public Climber() {
    climberLeft = new SparkMax(ClimberConstants.LEFT_SPARK_ID, MotorType.kBrushless);
    REVEncoder = new DutyCycleEncoder(ClimberConstants.CLIMBER_ENCODER);
    
    climberGlobal = new SparkMaxConfig();
    climberLeftConfig = new SparkMaxConfig();
    climberRightConfig = new SparkMaxConfig();

    climberGlobal
      .smartCurrentLimit(80)
      .idleMode(IdleMode.kBrake);

    climberLeftConfig
      .apply(climberGlobal)
      .inverted(true)
      .smartCurrentLimit(80)
      
      .encoder
      .positionConversionFactor(225);


    climberRightConfig
      .apply(climberGlobal)
      .follow(climberLeft, true)
      .smartCurrentLimit(80)
      .encoder
        .positionConversionFactor(225);


    climberLeft.configure(climberLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  public Command climb(DoubleSupplier velocity){
    return run(()->{
      
      if((getREVEncoderValue()>0.98 && velocity.getAsDouble() > 0) || 
        (getREVEncoderValue()<0.67 && velocity.getAsDouble() < 0 )){
        climberLeft.set(0);
      } else {
        climberLeft.set(velocity.getAsDouble()*14);
      }
      
      //climberLeft.setVoltage(velocity.getAsDouble()*14);
    });
  }

  public Command stopClimb() {
    return runOnce(()->climberLeft.set(0));
  }

  public double getEncoderValue() {
    return climberEncoder.getPosition();
  }
  public double getREVEncoderValue(){
    return REVEncoder.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Rev Encoder", getREVEncoderValue());
    //SmartDashboard.putNumber("Climber Rotate Position", climberEncoder.getPosition());
  }
}
