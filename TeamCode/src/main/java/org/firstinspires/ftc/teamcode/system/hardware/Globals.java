package org.firstinspires.ftc.teamcode.system.hardware;

public class Globals {
    // All constants

    // Use in Auto
    public static boolean
            BLUE_AUTO = false,
            RED_AUTO = false;

    // Inverse kinematic values
    public static double
            APRILTAG_POSE_X, // Apriltag coordinates from camera
            APRILTAG_POSE_Y,
            APRILTAG_POSE_Z,

            PIXEL_DISTANCE_X = 0, // Distance from apriltag to pixel scoring
            PIXEL_DISTANCE_Z = 12,

            PIXEL_PITCH_DEGREES = 30, // Scoring pixel pitch (Backdrop pitch)
            PIXEL_PITCH_RADIANS = Math.toRadians(PIXEL_PITCH_DEGREES),

            APIXEL_DISTANCE_Y = PIXEL_DISTANCE_Z *Math.tan(PIXEL_PITCH_RADIANS),

            PIXEL_POSE_X = APRILTAG_POSE_X + PIXEL_DISTANCE_X, // Pixel scoring coordinates
            PIXEL_POSE_Y = APRILTAG_POSE_Y + APIXEL_DISTANCE_Y,
            PIXEL_POSE_Z = APRILTAG_POSE_Z + PIXEL_DISTANCE_Z;


}
