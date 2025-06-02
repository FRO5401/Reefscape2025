// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.InfeedConstants.IntakeConstants;

public class Manipulator extends SubsystemBase {
    /***  Rev ***/
    /*      Motors */
    private final SparkMax pivot = new SparkMax(InfeedConstants.PivotConstants.PIVOT_ID, MotorType.kBrushless);

    private final SparkMax intakeLeft = new SparkMax(InfeedConstants.IntakeConstants.INTAKE_MOTOR_LEFT, MotorType.kBrushless);
    private final SparkMax intakeRight = new SparkMax(InfeedConstants.IntakeConstants.INTAKE_MOTOR_RIGHT, MotorType.kBrushless);

    private final SparkMax rotateLeft = new SparkMax(InfeedConstants.IntakeConstants.ROTATE_MOTOR_LEFT, MotorType.kBrushless);
    private final SparkMax rotateRight = new SparkMax(InfeedConstants.IntakeConstants.ROTATE_MOTOR_RIGHT, MotorType.kBrushless);

    /*      Encoders */
    private RelativeEncoder pivotEncoeer = pivot.getEncoder();

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
    private DigitalInput beamBreak = new DigitalInput(InfeedConstants.BEAM_BREAK_ID);;

    //.02 is the schedular cycle time, 
    private Debouncer currentFilter = new Debouncer(.15, DebounceType.kBoth); 

    /** Creates a new Maniuplator. */
    public Manipulator() {

        globalConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .disableFollowerMode();

        intakeLeftConfig
            .apply(globalConfig)
            .inverted(false);
    
        intakeRightConfig
            .apply(globalConfig)
            .inverted(true);

        rotateLeftConfig
            .apply(globalConfig)
            .inverted(false)
            .disableFollowerMode()
            .smartCurrentLimit(20)
            .encoder
                .positionConversionFactor(16);
      
        rotateRightConfig
            .apply(globalConfig)
            .inverted(false)
            .disableFollowerMode()
            .smartCurrentLimit(20)
            .encoder
                .positionConversionFactor(16);

        pivotConfig
            .apply(globalConfig)
            .inverted(true)
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake)
            .encoder
                .positionConversionFactor(13.8);
      
        rotateLeftConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(InfeedConstants.IntakeConstants.ROTATE_KP)
            .i(InfeedConstants.IntakeConstants.ROTATE_kI)
            .d(InfeedConstants.IntakeConstants.ROTATE_kD);

        rotateRightConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(InfeedConstants.IntakeConstants.ROTATE_KP)
            .i(InfeedConstants.IntakeConstants.ROTATE_kI)
            .d(InfeedConstants.IntakeConstants.ROTATE_kD);

        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(InfeedConstants.PivotConstants.PIVOT_KP)
            .i(InfeedConstants.PivotConstants.PIVOT_KI)
            .d(InfeedConstants.PivotConstants.PIVOT_KD);
      
        intakeLeft.configure(intakeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeRight.configure(intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rotateLeft.configure(rotateLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rotateRight.configure(rotateRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /*  Encoder Starting Position */
        rotateLeftEncoder.setPosition(0);
        rotateRightEncoder.setPosition(0);

        pivotEncoeer.setPosition(0);
    }

    public Command zeroPinch(){
        return runOnce(()->{
            rotateLeftEncoder.setPosition(0);
            pivotEncoeer.setPosition(0);
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
                Map.entry(ElevatorConstants.BARGE, setVelocity(IntakeConstants.TELEOP_REPEL_ALGEA, 0)),
                Map.entry(ElevatorConstants.L4, setVelocity(()->IntakeConstants.TELEOP_REPEL_CORAL)),
                Map.entry(ElevatorConstants.L3, setVelocity(()->IntakeConstants.TELEOP_REPEL_CORAL)),
                Map.entry(ElevatorConstants.L2, setVelocity(()->IntakeConstants.TELEOP_REPEL_CORAL)),
                Map.entry(ElevatorConstants.PROCESSOR, setVelocity(()->-0.2)),
                Map.entry(ElevatorConstants.FLOOR, setVelocity(()->-0.2))
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
        return ()->currentFilter.calculate((leftIntakeCurrent()>21));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("HasCoral", getBeamBreak());
        SmartDashboard.putBoolean("HasAlgea", isCurrentSpiked().getAsBoolean());
        SmartDashboard.putNumber("Pivot Current", pivot.getOutputCurrent());

  }
}
