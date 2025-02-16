// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Amps;

// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.GravityTypeValue;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Constants.ElevatorConstants;

// public class Elevator extends SubsystemBase {
//   private TalonFX elevator;
//   Slot0Configs slot0Configs;
//   double kS;
//   double kP;
//   double kV;
//   double kD;
//   double kI;
//   PositionVoltage PositionPID;

//   /** Creates a new Elevator. */
//   public Elevator() {
//     elevator = new TalonFX(ElevatorConstants.elevatorID);
//     elevator.setNeutralMode(NeutralModeValue.Brake);


//     Slot0Configs slot0Configs = new Slot0Configs();
//     TalonFXConfiguration config = new TalonFXConfiguration();
//     config.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(80)).withSupplyCurrentLimit(Amps.of(80)));
//     config.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
//     slot0Configs.kP = Constants.ElevatorConstants.KP; // An error of 1 rotation results in 2.4 V output
//     slot0Configs.kI = Constants.ElevatorConstants.KI; // no output for integrated error
//     slot0Configs.kD = Constants.ElevatorConstants.KD; // A velocity of 1 rps results in 0.1 V output
//     slot0Configs.kA=Constants.ElevatorConstants.KA;
//     slot0Configs.kV=Constants.ElevatorConstants.KV;
//     slot0Configs.kG=Constants.ElevatorConstants.KG;
//     slot0Configs.GravityType=GravityTypeValue.Elevator_Static;
    

//     elevator.getConfigurator().apply(slot0Configs);
//     elevator.getConfigurator().apply(config);

//     PositionPID = new PositionVoltage(0).withSlot(0);
//     elevator.getConfigurator().apply(slot0Configs);
//     elevator.setPosition(0);


//   }
//   public void moveElevator(double speed){
//     elevator.set(speed);
//   }

//   @Override
//   public void periodic() {

//     // This method will be called once per scheduler run
//   }
//   public Command setPosition(double pose) {
//     // Inline construction of command goes here.
//     // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return runOnce(
//         () -> {
//           elevator.setControl(PositionPID.withPosition(pose));
//         });
//   }
// }
