package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignAndDriveToReef extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    Debouncer finished = new Debouncer(.15);

    private final PIDController thetaController = new PIDController(3, 1, 0.1);
    private final ProfiledPIDController yController = new ProfiledPIDController(3.5, 1, 0.2,
            new TrapezoidProfile.Constraints(Constants.Swerve.MaxSpeed, .5));
    private final ProfiledPIDController xController = new ProfiledPIDController(3.5, 1, 0.2,
            new TrapezoidProfile.Constraints(Constants.Swerve.MaxSpeed, .5));
    private final Pose2d targetPose;
    private final double offset;
    private final double tolerance;
    private final Rotation2d rotationOffset;
    Transform2d poseOffset;

    public AlignAndDriveToReef(
            CommandSwerveDrivetrain drivetrain,
            double alignmentOffset,
            Pose2d alignmentPose,
            Rotation2d rotationOffset,
            double tolerance) {
        this.drivetrain = drivetrain;
        this.offset = alignmentOffset;
        this.targetPose = alignmentPose;
        this.rotationOffset = rotationOffset;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {

        // Camera id
        // tagId
        // Rotation to face the tag

        thetaController.setSetpoint(rotationOffset.getRadians());
        yController.setGoal(offset);
        xController.setGoal((0));
        thetaController.enableContinuousInput(0, 2 * Math.PI);
        thetaController.setTolerance(Units.degreesToRadians(2));
        yController.setTolerance(tolerance);
        xController.setTolerance(tolerance);
        yController.setIZone(.05);
        xController.setIZone(.05);
        thetaController.setIZone(Units.degreesToRadians(2) * 5);
    }

    @Override
    public void execute() {
        // double currentHeading =
        // drivetrain.getState().Pose.getRotation().getRadians();
        Pose2d currentPose = drivetrain.getState().Pose;

        poseOffset = currentPose.minus(targetPose);

        double thetaVelocity = thetaController.calculate(poseOffset.getRotation().getRadians());

        double yVelocityController = yController.calculate(poseOffset.getY());

        double xVelocityController = xController.calculate(poseOffset.getX());

        Rotation2d tagRotation = targetPose.getRotation();

        ChassisSpeeds tagRelativeCommandedVelocities = new ChassisSpeeds();

        tagRelativeCommandedVelocities.vyMetersPerSecond = yVelocityController;
        tagRelativeCommandedVelocities.omegaRadiansPerSecond = thetaVelocity;
        tagRelativeCommandedVelocities.vxMetersPerSecond = xVelocityController;

        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(tagRelativeCommandedVelocities,
                tagRotation);

        // System.out.println(offset.getRotation().getRadians());
        drivetrain.setControl(drivetrain.driveFieldRelative(fieldRelativeSpeeds));

        SmartDashboard.putNumber("x Offset", Units.metersToInches(poseOffset.getX()));
        SmartDashboard.putNumber("y Offset", Units.metersToInches(poseOffset.getY()));
        SmartDashboard.putNumber("theta", Units.metersToInches(poseOffset.getRotation().getDegrees()));
        SmartDashboard.putNumber("Tolerance", tolerance);
        SmartDashboard.putBooleanArray("COmmand end ",
                new Boolean[] { Math.abs(xController.getPositionError()) < tolerance,
                        Math.abs(yController.getPositionError()) < tolerance,
                        (Math.abs(thetaController.getError()) < tolerance) });

    }

    @Override
    public boolean isFinished() {
        return finished.calculate(atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {

        drivetrain.setControl(drivetrain.getRobotCentricRequest()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));

    }

    public boolean atSetpoint() {
        return Math.abs(xController.getPositionError()) < tolerance
                && Math.abs(yController.getPositionError()) < tolerance
                && (Math.abs(thetaController.getError()) < Units.degreesToRadians(2));
    }
}
