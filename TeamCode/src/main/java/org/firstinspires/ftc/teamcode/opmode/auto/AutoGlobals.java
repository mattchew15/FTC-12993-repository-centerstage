package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoGlobals {

    public static double

        xPosition,
        yPosition,
        headingPosition,

        SlowerVelocityConstraintIntake = 15,
        SlowerVelocityConstraintDeposit = 15,
        SlowerVelocityConstraintDepositFaster = 25,

        StartPoseYBackdrop = -59,
        StartPoseXBackdrop = 12,
        StartPoseHeadingBackdrop = Math.toRadians(180),

        FrontPreloadY = -37,

        PreloadPose1X = 35,
        PreloadPose1Y = -32,
        PreloadPose1Heading = Math.toRadians(180),

        PreloadPose2X = 35,
        PreloadPose2Y = -35,
        PreloadPose2Heading = Math.toRadians(175),

        PreloadPose3X = 35,
        PreloadPose3Y = -38,
        PreloadPose3Heading = Math.toRadians(170),

        MiddleLaneY = -31,
        LaneOffset = 29,

        ParkX = 45,
        ParkMiddleY = -14,
        ParkWallY = -54;


}
