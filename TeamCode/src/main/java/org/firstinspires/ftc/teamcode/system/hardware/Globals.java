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

            GAMEPAD_TRIGGER_THRESHOLD = 0.2,
            A; // angle offset for autonmous

    public static int
            SIXTY_DEGREE_TICKS = 0,
            UPRIGHT_PITCH_TICKS = -300,
            PITCH_LOW_DEGREE_TICKS = 1100,
            PITCH_MID_DEGREE_TICKS = 400,

            INTAKE_SLIDE_EXTENDO_TELEOP_FAR = 690,
            INTAKE_SLIDE_EXTENDO_TELEOP_CLOSE = 410,

            LIFT_HIGH_POSITION_TICKS = 600,
            LIFT_MEDIUM_POSITION_TICKS = 400,
            LIFT_LOW_POSITION_TICKS = 70,

            LIFT_MEDIUM_POSITION_PITCHING_TICKS = 580,
            LIFT_LOW_POSITION_PITCHING_TICKS = 690,
            LIFT_HITS_WHILE_PITCHING_THRESHOLD = 110,

            S; // the side multiplier to change sides for autonomous

    public static double degreestoTicksPitchMotor(double degrees){
        return degrees * PITCH_TICKS_PER_DEGREES;
    }

    public static double ticksToDegreePitchMotor (double ticks){
        return ticks/PITCH_TICKS_PER_DEGREES;
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

    public static double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }

}
