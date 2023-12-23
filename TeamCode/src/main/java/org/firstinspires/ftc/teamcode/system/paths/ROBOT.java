package org.firstinspires.ftc.teamcode.system.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class ROBOT
{
    private static Pose2d currentPose = new Pose2d();

    public void setPose2d(Pose2d pose)
    {
        currentPose = pose;
    }

    public Pose2d getPose2d()
    {
        return currentPose;
    }
public static double

    worldXPosition = 0,
    worldYPosition = 0,
    worldThetaPosition = 0; // in rad

public static double
    powerLB = 0,
    powerLF= 0,
    powerRB = 0,
    powerRF= 0;
public static double
    movementX = 0,
    movementY = 0,
    movementTurn = 0;
}
