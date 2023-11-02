package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.system.vision.ContourBlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.ContourRedTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.YCrCbBlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.YCrCbRedTeamPropDetectorPipeline;

@Config // Allows dashboard tune
public class Globals {

    // Auto Constants
    public static double
            autoTimer,
            stateTimer,
            numCycles = 0;

    // Auto States
    public static boolean
            BLUE_AUTO = false,
            RED_AUTO = false,
            FRONT_AUTO = false,
            BACK_AUTO = false;

    // Pitch
    public static double
            PITCH_TICKS_PER_REVOLUTION = 3895.9,
            PITCH_TICKS_PER_DEGREES = PITCH_TICKS_PER_REVOLUTION / 360,

            GAMEPAD_TRIGGER_THRESHOLD = 0.2;

    public static int
            SIXTY_DEGREE_TICKS = 400,
            PITCH_LOW_DEGREE_TICKS = 600, //guesses at this point
            PITCH_MID_DEGREE_TICKS = 500,

            INTAKE_SLIDE_EXTENDO_TELEOP = 500,

            LIFT_HIGH_POSITION_TICKS = 800,
            LIFT_MEDIUM_POSITION_TICKS = 450,
            LIFT_LOW_POSITION_TICKS = 100,

            LIFT_MEDIUM_POSITION_PITCHING_TICKS = 580,
            LIFT_LOW_POSITION_PITCHING_TICKS = 300,
            LIFT_HITS_WHILE_PITCHING_THRESHOLD = 200;


    public static double degreestoTicksPitchMotor(double degrees){
        return degrees * PITCH_TICKS_PER_DEGREES;
    }

    // Inverse Kinematic Constants
    public static double
            APRILTAG_POSE_X, // Apriltag coordinates from camera
            APRILTAG_POSE_Y,
            APRILTAG_POSE_Z,

            PIXEL_DISTANCE_X = 0, // Distance from apriltag to pixel scoring
            PIXEL_DISTANCE_Z = 12,

            PIXEL_PITCH_DEGREES = 30, // Scoring pixel pitch (Backdrop pitch)
            PIXEL_PITCH_RADIANS = Math.toRadians(PIXEL_PITCH_DEGREES),
            PIXEL_DISTANCE_Y = PIXEL_DISTANCE_Z *Math.tan(PIXEL_PITCH_RADIANS),

            PIXEL_POSE_X, // Pixel scoring coordinates
            PIXEL_POSE_Y,
            PIXEL_POSE_Z;

}
