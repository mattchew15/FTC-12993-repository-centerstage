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

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.accessory.PID;

import java.util.ArrayList;
import java.util.Arrays;



public class DriveTrainKinematics
{
    private static PID TRANSLATIONAL_PID = new PID(0.016, 0, 0.00000, 0 , 0);
    private static PID HEADING_PID = new PID(0.6, 0.0008, 0.024, 312 , 0);


    public static double PID_X;
    public static double PID_Y;
    public static double unwrappedPID_H;
    public static double wrappedPID_H;

    public static double x_Rotated;
    public static double y_Rotated;
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
    public static double[] power()
    {
        double y = - movement_y;
        double t = - movement_turn;

        double xRotated = movement_x * cos(worldAngle_rad) - y * sin(worldAngle_rad);
        double yRotated = movement_x * sin(worldAngle_rad) + y * cos(worldAngle_rad);
        x_Rotated = xRotated;
        y_Rotated = yRotated;

        double leftF = MathUtils.clamp(xRotated + yRotated + t, -1, 1);
        double rightF = MathUtils.clamp(xRotated - yRotated - t, -1, 1);
        double leftB = MathUtils.clamp(xRotated - yRotated + t, -1, 1);
        double rightB = MathUtils.clamp(xRotated + yRotated - t, -1, 1);

        return new double[]{leftF,rightF,leftB,rightB};
    }
    public static double[] powerDebug(double movement_x, double movement_y, double movement_turn)
    {
        double x = movement_x;
        double y = - movement_y;
        double t = - movement_turn;

        // TODO: fix the coordinateS AAAAAAAAA
        //double xRotated = movement_x * cos(angle) - y * sin(angle);
        //double yRotated = movement_x * sin(angle) + y * cos(angle);
       // x_Rotated = xRotated;
        //y_Rotated = yRotated;

        double leftF = MathUtils.clamp(x + y + t, -1, 1);
        double rightF = MathUtils.clamp(x - y - t, -1, 1);
        double leftB = MathUtils.clamp(x - y + t, -1, 1);
        double rightB = MathUtils.clamp(x + y - t, -1, 1);

        return new double[]{leftF,rightF,leftB,rightB};
    }
    public static double[] powerPID(double targetX, double targetY, double targetH)
    {
        // the world is not necessary but makes it more readable for now
        double x = TRANSLATIONAL_PID.update(targetX, worldXPosition, 99999);
        double y = - TRANSLATIONAL_PID.update(targetY, worldYPosition, 99999);
        double t = HEADING_PID.update(AngleWrap(targetH), worldAngle_rad, 2 * PI);

        PID_X = x;
        PID_Y = y;
        unwrappedPID_H = t;
        t = AngleWrap(t); // TODO evalute the removal of this... it can be...
        wrappedPID_H = t;
        double xRotated = x * cos(worldAngle_rad) - y * sin(worldAngle_rad);
        double yRotated = x * sin(worldAngle_rad) + y * cos(worldAngle_rad);
        x_Rotated = xRotated;
        y_Rotated = yRotated;

        double leftF = MathUtils.clamp(xRotated + yRotated + t, -1, 1);
        double rightF = MathUtils.clamp(xRotated - yRotated - t, -1, 1);
        double leftB = MathUtils.clamp(xRotated - yRotated + t, -1, 1);
        double rightB = MathUtils.clamp(xRotated + yRotated - t, -1, 1);

        return new double[]{leftF,rightF,leftB,rightB};
    }
    public static double[] driveToPosition(double targetX, double targetY, double targetHeading, double robotX, double robotY, double robotTheta, Telemetry telemetry)
    {
        double x = TRANSLATIONAL_PID.update(targetX, robotX, 1);
        double y = -TRANSLATIONAL_PID.update(targetY, robotY, 1);
        double theta = -HEADING_PID.updateWithError(AngleWrap(targetHeading -robotTheta) , 1);
        telemetry.addData("Pid theta: ", theta);
        telemetry.addData("Pid X: ", x);
        telemetry.addData("Pid Y: ", y);

        double x_rotated = (x * Math.cos(robotTheta) - y * Math.sin(robotTheta));
        double y_rotated = (x * Math.sin(robotTheta) + y * Math.cos(robotTheta));
        telemetry.addData("Y rotated: ", y_rotated);
        telemetry.addData("X rotated: ", x_rotated);

        double FL = MathUtils.clamp(x_rotated + y_rotated + theta, -1, 1);
        double BL = MathUtils.clamp(x_rotated - y_rotated + theta, -1, 1);
        double FR = MathUtils.clamp(x_rotated - y_rotated - theta, -1, 1);
        double BR = MathUtils.clamp(x_rotated + y_rotated - theta, -1, 1);

        telemetry.addData("FL ", FL);
        telemetry.addData("BL ", BL);
        telemetry.addData("FR ", FR);
        telemetry.addData("BR ", BR);

        return new double[]{FL, FR, BL, BR};
    }
    public static double[] UNCLAMPED_PowerPID()
    {
        // the world is not necessary but makes it more readable for now
        double x = TRANSLATIONAL_PID.update(movement_x + worldXPosition, worldXPosition, 99999);
        double y = TRANSLATIONAL_PID.update(movement_y + worldYPosition, worldYPosition, 99999);
        double t = HEADING_PID.update(AngleWrap(movement_turn + worldAngle_rad), worldAngle_rad, 2 * PI);

        PID_X = x;
        PID_Y = y;
        unwrappedPID_H = t;
        t = AngleWrap(t); // TODO evalute the removal of this... it can be...
        wrappedPID_H = t;
        double xRotated = x * cos(worldAngle_rad) - y * sin(worldAngle_rad);
        double yRotated = x * sin(worldAngle_rad) + y * cos(worldAngle_rad);

        double leftF = xRotated + yRotated + t;
        double rightF = xRotated - yRotated - t;
        double leftB = xRotated - yRotated + t;
        double rightB = xRotated + yRotated - t;

        return new double[]{leftF,rightF,leftB,rightB};
    }
    public static double[] normalPID()
    {
        double maxPower = 1;

        // the world is not necessary but makes it more readable for now
        double x = TRANSLATIONAL_PID.update(movement_x , 0, 99999);
        double y = TRANSLATIONAL_PID.update(movement_y , 0, 99999);
        double t = HEADING_PID.update(AngleWrap(movement_turn), 0, 2 * PI);
        t = AngleWrap(t);
        double xRotated = x * cos(worldAngle_rad) - y * sin(worldAngle_rad);
        double yRotated = x * sin(worldAngle_rad) + y * cos(worldAngle_rad);

        double[] aws = new double[]{xRotated + yRotated + t, xRotated - yRotated - t, xRotated - yRotated + t, xRotated + yRotated - t};

        for (double power: aws)
        {
            if (abs(power) > maxPower)
            {
                maxPower = abs(power);
            }
        }
        for (int i = 0; i < aws.length; i++)
        {
            aws[i] /= maxPower;
        }
        return aws;


    }
}


