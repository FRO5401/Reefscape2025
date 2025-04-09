// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveUtils {
    private static final PIDController rotate = new PIDController(.04, 0, 0.005);
    public static void setupUtil(){
        rotate.enableContinuousInput(0, 360);
    }
    public static double rotationPoint(double angle, double yaw){
        SmartDashboard.putNumberArray("Utils/Swerve", new double[]{angle, yaw, rotate.calculate(yaw)});
        rotate.setSetpoint(angle);
        
        if(Math.abs(angle)> .1){
            return rotate.calculate(yaw);
        } else {
            return 0;
        }
        
        
    }
}
