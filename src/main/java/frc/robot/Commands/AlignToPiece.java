package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToPiece extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    private final DoubleSupplier xVelocity;
    private final DoubleSupplier yVelocity;

    private final PIDController thetaController = new PIDController(4, 0, 0);
    private final double targetPose;
    private final Rotation2d rotationOffset;


    public AlignToPiece(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xVelocity,
            DoubleSupplier yVelocity,
            double alignmentOffset,
            double alignmentPose,
            Rotation2d rotationOffset) {
        this.drivetrain = drivetrain;
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.targetPose = alignmentPose;
        this.rotationOffset = rotationOffset;
    }

    @Override
    public void initialize() {

        // Camera id
        // tagId
        // Rotation to face the tag

        thetaController.setSetpoint(rotationOffset.getRadians());
        thetaController.setSetpoint(0);
        thetaController.enableContinuousInput(0, 2 * Math.PI);
        thetaController.setTolerance(Units.degreesToRadians(1));
    }

    @Override
    public void execute() {
        // double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();

        

        double thetaVelocity = thetaController.calculate(targetPose);
        if (thetaController.atSetpoint()) {
            thetaVelocity = 0;
        }
        

        Rotation2d tagRotation = new Rotation2d(Units.degreesToRadians(targetPose));

        ChassisSpeeds driverCommandedVelocities =
                new ChassisSpeeds(xVelocity.getAsDouble() , yVelocity.getAsDouble(), 0);

        ChassisSpeeds fieldCommandedVelocities =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        driverCommandedVelocities, drivetrain.getPose().getRotation());

        ChassisSpeeds tagRelativeCommandedVelocities =
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldCommandedVelocities, tagRotation);

        tagRelativeCommandedVelocities.omegaRadiansPerSecond = thetaVelocity;

        ChassisSpeeds fieldRelativeSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(tagRelativeCommandedVelocities, tagRotation);

        // System.out.println(offset.getRotation().getRadians());
        drivetrain.setControl(drivetrain.driveFieldRelative(fieldRelativeSpeeds));
    }

    @Override
    public boolean isFinished() {
        return false;
        // return thetaController.atSetpoint() && yController.atSetpoint();
    }
}