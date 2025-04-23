// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretConstants.TurretState;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  TurretConstants.TurretState currentState =TurretState.Targeting;

  TalonFX rotateMotor = new TalonFX(25);

  TalonFXConfiguration turretConfig = new TalonFXConfiguration();

  PositionVoltage positionRequest;
  VoltageOut voltageRequest;
  MotionMagicVoltage motionMagicRequest;

  Pose2d target;

  Supplier<Pose2d> robotPose;
  Transform2d poseOffset;

  double currentAngle= 0;

  public Turret(Supplier<Pose2d> robotPose) {
    
    configure();

    this.robotPose = robotPose;

    positionRequest = new PositionVoltage(0);
    voltageRequest = new VoltageOut(0);
    motionMagicRequest = new MotionMagicVoltage(0);
  }

  public void configure() {
    turretConfig.Slot0.kS = .25;
    turretConfig.Slot0.kV = 0;
    turretConfig.Slot0.kA = .1;
    turretConfig.Slot0.kP = 150;
    turretConfig.Slot0.kI = 0;
    turretConfig.Slot0.kD = 5;

    turretConfig.MotionMagic.MotionMagicCruiseVelocity = 160; // rps
    turretConfig.MotionMagic.MotionMagicAcceleration = 160; // rps/s
    turretConfig.MotionMagic.MotionMagicJerk = 1600; // rps/s/s





    turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turretConfig.CurrentLimits.SupplyCurrentLimit = 30;


    rotateMotor.getConfigurator().apply(turretConfig);
  }

  @Override
  public void periodic() {
    if(robotPose != null && target != null && currentState == TurretState.Targeting){
      poseOffset = robotPose.get().minus(target);
      setTurretAngle(Math.atan2(poseOffset.getY(), poseOffset.getX())+poseOffset.getRotation().getRadians()+Math.PI, robotPose.get());
    } 
    SmartDashboard.putNumberArray("Turret pose", new double[]{robotPose.get().getX(), robotPose.get().getY()} );
    SmartDashboard.putNumber("Turret Angle", currentAngle-robotPose.get().getRotation().getRadians());
  }

    /**
   * Sets the speed of the turret motor
   * 
   * @param speed The speed to set the turret motor to (-1 to 1)
   */
  public void setTurretSpeed(double speed) {
    rotateMotor.set(speed);
  }

  /**
   * Sets the physical angle of the turret
   * 
   * @param angle The angle to set the turret to. <b> Units: </b> Radians
   * @param pose The pose of the robot, used for smartdashboard testing
   */
  private void setTurretAngle(double angle,Pose2d pose) {
    
    currentAngle = angle;
    rotateMotor.setControl(motionMagicRequest.withPosition(Units.radiansToRotations(angle)));
  }

  public Command setTurret(){
    return Commands.runOnce(()->{
      currentState = TurretState.Intaking;
      setTurretAngle(0, robotPose.get());
    });
  }

  public void setTarget(Pose2d target){
    currentState = TurretState.Targeting;
    this.target = target;
  }

  public Command setTargetCommand(Pose2d target){
    return Commands.runOnce(()->setTarget(target));
  }
}
