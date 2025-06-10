package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;

public final class ElevatorConstants {
    public static final int elevatorID = 13;
    public static final double SPEED_MODIFIER = 166;
    public static final double BARGE = 150 - 5;// -5
    public static final double L4 = 140 - 7;// -9
    public static final double L3 = 82 - 5;// -5

    public static final double L2 = 53.91748046875 - 6;// -6
    // TODO Fix this pls
    public static final double STATION = 52 - 7;// -7 at competition field
    public static final double PROCESSOR = 5;
    public static final double FLOOR = 1;

    public static final double KP = Robot.isReal() ? 4:4; // An error of 1 rotation results in 2.4 V output
    public static final double KI = 3; // no output for integrated error
    public static final double KD = .4; // A velocity of 1 rps results in 0.1 V output
    public static final double KA = .1;
    public static final double KV = .1;
    public static final double KG = 2.4;

}
