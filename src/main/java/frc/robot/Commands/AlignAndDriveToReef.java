package frc.robot.Commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

    private final PIDController thetaController = new PIDController(3, 1, 0);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 6, 0,new TrapezoidProfile.Constraints(Constants.Swerve.MaxSpeed, .5));
    private final ProfiledPIDController xController = new ProfiledPIDController(3, 6,  0,new TrapezoidProfile.Constraints(Constants.Swerve.MaxSpeed, .5));
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
        thetaController.enableContinuousInput(0, 2 * Math.PI);
        thetaController.setTolerance(Units.degreesToRadians(2));
        yController.setTolerance(tolerance);
        xController.setTolerance(tolerance);
        yController.setIZone(.05);
        xController.setIZone(.05) ;
        thetaController.setIZone(Units.degreesToRadians(2)*5);
    }

    @Override
    public void execute() {
        // double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
        Pose2d currentPose = drivetrain.getState().Pose;

        poseOffset = currentPose.minus(targetPose);

        double thetaVelocity = thetaController.calculate(poseOffset.getRotation().getRadians());
        if (Math.abs(thetaController.getError()) < Units.degreesToRadians(2)) {
            thetaVelocity = 0;
        }
        double yVelocityController = yController.calculate(poseOffset.getY());
        if (Math.abs(yController.getPositionError()) < tolerance) {
            yVelocityController = 0;
        }
        double xVelocityController = xController.calculate(poseOffset.getX());
        if (Math.abs(xController.getPositionError()) <  tolerance) {
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
        
        SmartDashboard.putNumber("x Offset", xController.getPositionError());
        SmartDashboard.putNumber("y Offset", yController.getPositionError());
        SmartDashboard.putNumber("theta", thetaController.getError());
        SmartDashboard.putBooleanArray("COmmand end ", new Boolean[]{Math.abs(xController.getPositionError()) <  tolerance ,Math.abs(yController.getPositionError()) <  tolerance,  (Math.abs(thetaController.getError()) <  tolerance)});

    }

    @Override
    public boolean isFinished() {

        return Math.abs(xController.getPositionError()) <  tolerance && Math.abs(yController.getPositionError()) <  tolerance && (Math.abs(thetaController.getError()) <  Units.degreesToRadians(2));
    }
}
