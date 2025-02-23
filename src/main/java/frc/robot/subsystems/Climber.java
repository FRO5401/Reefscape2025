// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  SparkMax sparkMax;
  SparkMaxConfig climberGlobal;
  SparkMaxConfig climberPivotConfig;
  RelativeEncoder climberEncoder;

  public Climber() {
    sparkMax = new SparkMax(ClimberConstants.sparkID, MotorType.kBrushless);
    
    climberGlobal = new SparkMaxConfig();
    climberPivotConfig = new SparkMaxConfig();

    climberGlobal
      .smartCurrentLimit(80)
      .idleMode(IdleMode.kBrake);

    climberPivotConfig
      .apply(climberGlobal)
      .inverted(true)
      .smartCurrentLimit(80)
      .encoder
      .positionConversionFactor(225);


    sparkMax.configure(climberPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climberEncoder.setPosition(0);
  }

  public void climb(double speed) {
    sparkMax.set(speed);
  }

  public void stopClimb() {
    sparkMax.set(0);
  }

  public double getEncoderValue() {
    return climberEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Rotate Position", climberEncoder.getPosition());
  }
}
