package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignAndDriveToReef extends Command {
    private CommandSwerveDrivetrain drivetrain;

    private PIDController thetaController = new PIDController(6, 0, 0);
    private PIDController yController = new PIDController(5, 0, 0);
    private PIDController xController = new PIDController(1, 0, 0);
    private Pose2d targetPose;
    private double offset;
    private Rotation2d rotationOffset;

    public AlignAndDriveToReef(
            CommandSwerveDrivetrain drivetrain,
            double alignmentOffset,
            Pose2d alignmentPose,
            Rotation2d rotationOffset) {
        this.drivetrain = drivetrain;
        this.offset = alignmentOffset;
        this.targetPose = alignmentPose;
        this.rotationOffset = rotationOffset;
    }


    @Override
    public void initialize() {

        // Camera id
        // tagId
        // Rotation to face the tag
        

        thetaController.setSetpoint(rotationOffset.getRadians());
        yController.setSetpoint(offset);
        thetaController.enableContinuousInput(0, 2 * Math.PI);
        thetaController.setTolerance(Units.degreesToRadians(1));
        yController.setTolerance(Units.inchesToMeters(0.2));
        xController.setTolerance(Units.inchesToMeters(0.2));
    }

    @Override
    public void execute() {
        // double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
        Pose2d currentPose = drivetrain.getState().Pose;

        Transform2d offset = currentPose.minus(targetPose);

        double thetaVelocity = thetaController.calculate(offset.getRotation().getRadians());
        if (thetaController.atSetpoint()) {
            thetaVelocity = 0;
        }
        double yVelocityController = yController.calculate(offset.getY());
        if (yController.atSetpoint()) {
            yVelocityController = 0;
        }
        double xVelocityController = xController.calculate(offset.getX());
        if (xController.atSetpoint()) {
            xVelocityController = 0;
        }

        Rotation2d tagRotation = targetPose.getRotation();

        ChassisSpeeds tagRelativeCommandedVelocities = new ChassisSpeeds();

        tagRelativeCommandedVelocities.vyMetersPerSecond = yVelocityController;
        tagRelativeCommandedVelocities.omegaRadiansPerSecond = thetaVelocity;
        tagRelativeCommandedVelocities.vxMetersPerSecond = xVelocityController;

        ChassisSpeeds fieldRelativeSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(tagRelativeCommandedVelocities, tagRotation);

        // System.out.println(offset.getRotation().getRadians());
        drivetrain.setControl(drivetrain.driveFieldRelative(fieldRelativeSpeeds));
        
        SmartDashboard.putNumber("x Offset", offset.getX());
    }

    @Override
    public boolean isFinished() {
        return false;
        // return thetaController.atSetpoint() && yController.atSetpoint();
    }
}