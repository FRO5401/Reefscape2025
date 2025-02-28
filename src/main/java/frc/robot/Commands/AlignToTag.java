// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTag extends Command {
  /** Creates a new AlignToTag. */
  CommandSwerveDrivetrain drivetrain;
  PhotonCamera camera1;
  PhotonCamera camera2;
  DoubleSupplier strafeInput;

  /* turn and forward values */
  double turn, forward;

    /* Constants for aligning */
    private final double VISION_TURN_kP = 0.01;
    private final double VISION_DES_ANGLE_deg = 0.0;
    private final double VISION_STRAFE_kP = 0.5;
    private final double VISION_DES_RANGE_m = 1.25;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);  // 3/4 of a rotation per second max angular

    /* Drive Command  */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public AlignToTag(
    CommandSwerveDrivetrain m_drivetrain,
    PhotonCamera m_camera1,
    PhotonCamera m_camera2,
    DoubleSupplier m_strafeInput
  ) {

    drivetrain = m_drivetrain;
    camera1 = m_camera1;
    camera2 = m_camera2;
    strafeInput = m_strafeInput;

    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // Read in relevant data from the Camera
        double targetYaw = 0.0;
        double targetRange = 0.0;
        var results = camera1.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 7) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetRange =
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        0.5, // Measured with a tape measure, or in CAD.
                                        1.435, // From 2024 game manual for ID 7
                                        Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                                        Units.degreesToRadians(target.getPitch()));

                    }
                }
            }
        } 
        var results2 = camera2.getAllUnreadResults();
        if (!results2.isEmpty()&& results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result2 = results2.get(results.size() - 1);
            if (result2.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result2.getTargets()) {
                    if (target.getFiducialId() == 7) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetRange =
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        0.5, // Measured with a tape measure, or in CAD.
                                        1.435, // From 2024 game manual for ID 7
                                        Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                                        Units.degreesToRadians(target.getPitch()));

                    }
                }
            }
        }

        turn =
          (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP;
        forward =
          (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP;

        // Command drivetrain motors based on target speeds
        drivetrain.applyRequest(() ->
        drive.withVelocityX(-forward * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-strafeInput.getAsDouble() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-turn * MaxAngularRate) // Drive counterclockwise with negative X (left)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
