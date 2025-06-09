// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.ManipulatorConstants.IntakeConstants;
import frc.robot.Constants.ManipulatorConstants.PivotConstants;

public class Manipulator extends SubsystemBase {
    /***  Rev ***/
    /*      Motors */
    private final SparkMax pivot = new SparkMax(PivotConstants.PIVOT_ID, MotorType.kBrushless);

    private final SparkMax intakeLeft = new SparkMax(IntakeConstants.INTAKE_MOTOR_LEFT, MotorType.kBrushless);
    private final SparkMax intakeRight = new SparkMax(IntakeConstants.INTAKE_MOTOR_RIGHT, MotorType.kBrushless);

    private final SparkMax rotateLeft = new SparkMax(IntakeConstants.ROTATE_MOTOR_LEFT, MotorType.kBrushless);
    private final SparkMax rotateRight = new SparkMax(IntakeConstants.ROTATE_MOTOR_RIGHT, MotorType.kBrushless);

    /*      Encoders */
    private RelativeEncoder pivotEncoder = pivot.getEncoder();

    private RelativeEncoder rotateLeftEncoder = rotateLeft.getEncoder();
    private RelativeEncoder rotateRightEncoder = rotateRight.getEncoder();

    /*      PID */
    private SparkClosedLoopController PivotPID = pivot.getClosedLoopController();

    private SparkClosedLoopController rotateLeftPID = rotateLeft.getClosedLoopController();
    private SparkClosedLoopController rotateRightPID = rotateRight.getClosedLoopController();

    /*      Configs */
    private SparkMaxConfig globalConfig = new SparkMaxConfig();

    private SparkMaxConfig pivotConfig = new SparkMaxConfig();

    private SparkMaxConfig intakeLeftConfig = new SparkMaxConfig();
    private SparkMaxConfig intakeRightConfig = new SparkMaxConfig();

    private SparkMaxConfig rotateLeftConfig = new SparkMaxConfig();
    private SparkMaxConfig rotateRightConfig = new SparkMaxConfig();

    //  Beam Break for coral detection
    private DigitalInput beamBreak = new DigitalInput(ManipulatorConstants.BEAM_BREAK_ID);;

    //.02 is the schedular cycle time, 
    private Debouncer currentFilter = new Debouncer(.15, DebounceType.kBoth); 

    /** Creates a new Maniuplator. */
    public Manipulator() {

        globalConfig
            .idleMode(IdleMode.kBrake)
            .disableFollowerMode();

        intakeLeftConfig
            .apply(globalConfig)
            .smartCurrentLimit(IntakeConstants.MINION_STALL_CURRENT)
            .inverted(false);
    
        intakeRightConfig
            .apply(globalConfig)
            .smartCurrentLimit(IntakeConstants.MINION_STALL_CURRENT)
            .inverted(true);

        rotateLeftConfig
            .apply(globalConfig)
            .inverted(false)
            .disableFollowerMode()
            .smartCurrentLimit(IntakeConstants.NEO550_STALL_CURRENT)
            .encoder
                .positionConversionFactor(IntakeConstants.ROTATE_GEAR_RATIO);
      
        rotateRightConfig
            .apply(globalConfig)
            .inverted(false)
            .disableFollowerMode()
            .smartCurrentLimit(IntakeConstants.NEO550_STALL_CURRENT)
            .encoder
                .positionConversionFactor(IntakeConstants.ROTATE_GEAR_RATIO);

        pivotConfig
            .apply(globalConfig)
            .inverted(true)
            .smartCurrentLimit(PivotConstants.NEO1650_STALL_CURRENT)
            .idleMode(IdleMode.kBrake)
            .encoder
                .positionConversionFactor(PivotConstants.GEAR_RATIO);
      
        rotateLeftConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(IntakeConstants.ROTATE_KP)
            .i(IntakeConstants.ROTATE_kI)
            .d(IntakeConstants.ROTATE_kD);

        rotateRightConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(IntakeConstants.ROTATE_KP)
            .i(IntakeConstants.ROTATE_kI)
            .d(IntakeConstants.ROTATE_kD);

        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(PivotConstants.PIVOT_KP)
            .i(PivotConstants.PIVOT_KI)
            .d(PivotConstants.PIVOT_KD);
      
        intakeLeft.configure(intakeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeRight.configure(intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rotateLeft.configure(rotateLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rotateRight.configure(rotateRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /*  Encoder Starting Position */
        rotateLeftEncoder.setPosition(0);
        rotateRightEncoder.setPosition(0);

        pivotEncoder.setPosition(0);
    }

    public Command zeroManipulator(){
        return runOnce(()->{
            rotateLeftEncoder.setPosition(0);
            rotateRightEncoder.setPosition(0);
            pivotEncoder.setPosition(0);
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

    //overloaded to make it so that we can add backspin to the algea if needed
    public Command setVelocity(double leftVelocity, double rightVelocity){
        return runOnce(()->{
            intakeLeft.set(leftVelocity);
            intakeRight.set(rightVelocity);
        });
    }

    public Command setVelocity(DoubleSupplier velocity){
        return runOnce(()->{
            intakeLeft.set(velocity.getAsDouble());
            intakeRight.set(velocity.getAsDouble());
        });
    }

     //selects the command based off of elevator pose
    public Command expelCommand(Elevator elevator){
        return new SelectCommand<>(
            // Maps elevator state to different manipulator speeds
            Map.ofEntries(
                Map.entry(ElevatorConstants.BARGE, setVelocity(IntakeConstants.TELEOP_EXPEL_ALGEA, 0)),
                Map.entry(ElevatorConstants.L4, setVelocity(()->IntakeConstants.TELEOP_EXPEL_CORAL)),
                Map.entry(ElevatorConstants.L3, setVelocity(()->IntakeConstants.TELEOP_EXPEL_CORAL)),
                Map.entry(ElevatorConstants.L2, setVelocity(()->IntakeConstants.TELEOP_EXPEL_CORAL)),
                Map.entry(ElevatorConstants.PROCESSOR, setVelocity(()->IntakeConstants.LOW_ALGEA_EXPEL)),
                Map.entry(ElevatorConstants.FLOOR, setVelocity(()->IntakeConstants.LOW_ALGEA_EXPEL))
            ),
            elevator::getElevatorState
        );
    }

    public Command stopIntake(){
        return runOnce(()->{
            intakeLeft.set(0);
            intakeRight.set(0);
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

    public BooleanSupplier isCurrentSpiked(){
        return ()->currentFilter.calculate((leftIntakeCurrent()>IntakeConstants.ALGEA_CURRENT_SPIKE));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Manipulator/HasCoral", getBeamBreak());
        SmartDashboard.putBoolean("Manipulator/HasAlgea", isCurrentSpiked().getAsBoolean());
        SmartDashboard.putNumber("Manipulator/Pivot-Current", pivot.getOutputCurrent());
        Logger.recordOutput("Manipulator/Pivot-Position", pivotEncoder.getPosition());
  }
}
