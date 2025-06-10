package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ElevatorReal extends Elevator {
    private static final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.elevatorID);
    private static Slot0Configs slot0Configs = new Slot0Configs();
    private static TalonFXConfiguration config = new TalonFXConfiguration();
    private static PositionVoltage PositionPID  = new PositionVoltage(0).withSlot(0);

    public ElevatorReal(){
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);

        config.withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(120))
            .withSupplyCurrentLimit(Amps.of(80))
        );

        config.withMotorOutput(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
        );
        
        slot0Configs.kP = Constants.ElevatorConstants.KP; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = Constants.ElevatorConstants.KI; // no output for integrated error
        slot0Configs.kD = Constants.ElevatorConstants.KD; // A velocity of 1 rps results in 0.1 V output
        slot0Configs.kA=Constants.ElevatorConstants.KA;
        slot0Configs.kV=Constants.ElevatorConstants.KV;
        slot0Configs.kG=Constants.ElevatorConstants.KG;
        slot0Configs.GravityType=GravityTypeValue.Elevator_Static;

        elevatorMotor.getConfigurator().apply(slot0Configs);
        elevatorMotor.getConfigurator().apply(config);

        elevatorMotor.getConfigurator().apply(slot0Configs);
        elevatorMotor.setPosition(0);

    }

    @Override
    public Command setPosition(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }

    public void periodic(){

    }
}
