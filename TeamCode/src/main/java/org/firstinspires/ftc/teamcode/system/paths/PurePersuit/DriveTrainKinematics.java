package org.firstinspires.ftc.teamcode.system.paths.PurePersuit;

import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldYPosition;
import static java.lang.Math.*;


import androidx.core.math.MathUtils;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.system.accessory.PID;

import java.util.ArrayList;
import java.util.Arrays;



public class DriveTrainKinematics
{
    private static PID TRANSLATIONAL_PID = new PID(10, 0, 0, 0 , 0);
    private static PID HEADING_PID = new PID(8, 0, 0, 0 , 0);
    // Wheels vel
    public static double
    FR_Vel,
    FL_Vel,
    BR_Vel,
    BL_Vel;

    // Velocity of the robot
    public static double
    Forward_Vel,
    Strafe_Vel,
    Rotational_Vel; //rads/s

    public static double TRACK_WIDTH = 15.2 / 2; // GM0 uses radius but multiply by two always
    public static double TRACK_LENGTH = 10 / 2.0;
    public static double power = 0;
    public static double angleToPoint = 0;

    public static SimpleMatrix wheelsVelocity = new SimpleMatrix(new double[][]{
            {0},
            {0},
            {0},
            {0}
    });
    public static SimpleMatrix robotVelocity = new SimpleMatrix(new double[][]{
            {0},
            {0},
            {0}
    });
    public static double WHEELS_RADIUS = 1.88976;

    public static void updateRobotVelsGM0()
    {
        Forward_Vel = (FL_Vel+FR_Vel+BL_Vel+BR_Vel) / 4;
        Strafe_Vel = (FR_Vel + BL_Vel - FL_Vel - BR_Vel) / 4;
        Rotational_Vel = (FR_Vel + BR_Vel - BL_Vel - FL_Vel) / TRACK_WIDTH * 2;
    }
    public static void updateWheelsVelsGMO()
    {
        double w = 2 * TRACK_WIDTH * Rotational_Vel;
        FL_Vel = Forward_Vel - Strafe_Vel - w;
        BL_Vel = Forward_Vel + Strafe_Vel - w;
        BR_Vel = Forward_Vel - Strafe_Vel + w;
        FR_Vel = Forward_Vel + Strafe_Vel + w;
    }

    public static void updateRobotVelsRR()
    {
        double b = TRACK_WIDTH +TRACK_LENGTH;                                                                                           
        SimpleMatrix m = new SimpleMatrix(new double[][]{
                {1, -1, -b},
                {1, 1, -b},
                {1, -1, b},
                {1, 1, b}
        });
        m.scale(1/WHEELS_RADIUS);
        wheelsVelocity = m.mult(robotVelocity);
    }
    public static void updateWheelsVelsRR()
    {
        double b = TRACK_WIDTH +TRACK_LENGTH;
        SimpleMatrix m = new SimpleMatrix(new double[][]{
                {1, 1, 1, 1},
                {-1, 1, -1, 1},
                {-1/b, -1/b, 1/b, 1/b}
        });
        m.scale(WHEELS_RADIUS / 4);
        robotVelocity = m.mult(wheelsVelocity);
    }


    public static double[] powerFromVector()
    {
        double sin = sin(angleToPoint - PI / 4);
        double cos = cos(angleToPoint - PI / 4);
        double max = max(abs(sin), abs(cos));

        double leftF = MathUtils.clamp(power * cos / max + movement_turn, -1, 1);
        double rightF = MathUtils.clamp(power * sin / max - movement_turn, -1, 1);
        double leftB = MathUtils.clamp(power * cos / max + movement_turn, -1, 1);
        double rightB = MathUtils.clamp(power * sin / max - movement_turn, -1, 1);

        return new double[]{leftF,rightF,leftB,rightB};
    }
    public static double[] powerPID()
    {
        // the world is not necessary but makes it more readable for now
        double x = TRANSLATIONAL_PID.update(movement_x + worldXPosition, worldXPosition, 99999);
        double y = TRANSLATIONAL_PID.update(movement_y + worldYPosition, worldYPosition, 99999);
        double t = HEADING_PID.update(AngleWrap(movement_turn + worldAngle_rad), worldAngle_rad, 99999);
        double xRotated = x * cos(worldAngle_rad) - y * sin(worldAngle_rad);
        double yRotated = x * sin(worldAngle_rad) + y * cos(worldAngle_rad);

        double leftF = MathUtils.clamp(xRotated + yRotated + t, -1, 1);
        double rightF = MathUtils.clamp(xRotated - yRotated - t, -1, 1);
        double leftB = MathUtils.clamp(xRotated - yRotated + t, -1, 1);
        double rightB = MathUtils.clamp(xRotated + yRotated - t, -1, 1);

        return new double[]{leftF,rightF,leftB,rightB};
    }
}


