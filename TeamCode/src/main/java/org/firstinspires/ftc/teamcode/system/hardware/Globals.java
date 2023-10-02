package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.system.vision.BlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.Others.YCrCbBlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.Others.YCrCbRedTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.RedTeamPropDetectorPipeline;

@Config // Allows dashboard tune
public class Globals {
    // Auto States
    public static boolean
            BLUE_AUTO = false,
            RED_AUTO = false;

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
    public static BlueTeamPropDetectorPipeline.TeamPropPosition
            BLUE_LEFT = BlueTeamPropDetectorPipeline.TeamPropPosition.LEFT,
            BLUE_CENTER = BlueTeamPropDetectorPipeline.TeamPropPosition.CENTER,
            BLUE_RIGHT = BlueTeamPropDetectorPipeline.TeamPropPosition.RIGHT,
            BLUE_POSITION = null;
    public static RedTeamPropDetectorPipeline.TeamPropPosition
            RED_LEFT = RedTeamPropDetectorPipeline.TeamPropPosition.LEFT,
            RED_CENTER = RedTeamPropDetectorPipeline.TeamPropPosition.CENTER,
            RED_RIGHT = RedTeamPropDetectorPipeline.TeamPropPosition.RIGHT,
            RED_POSITION = null;

    //Intake Servo Positions
    public static double //give pos values
            BOTTOM_ROLLER_, //write states
            BRUSH_HEIGHT_,
            FLAP_CLOSE,
            FLAP_OPEN,
            INTAKE_LOCK_LOCK,
            INTAKE_LOCK_UNLOCK;

    //Outtake Servo Positions
    public static double //give pos values
            BOTTOM_ROLLER_5,
            BOTTOM_ROLLER_4,
            BOTTOM_ROLLER_3,
            BOTTOM_ROLLER_2,
            BOTTOM_ROLLER_1,
            BRUSH_HEIGHT_5,
            BRUSH_HEIGHT_4,
            BRUSH_HEIGHT_3,
            BRUSH_HEIGHT_2,
            BRUSH_HEIGHT_1,
            RAIL_LEFT,
            RAIL_CENTER,
            RAIL_RIGHT,
            ARM_LEFT_DOWN,
            ARM_LEFT_LIFT,
            ARM_LEFT_UP,
            ARM_RIGHT_DOWN,
            ARM_RIGHT_LIFT,
            ARM_RIGHT_UP,
            ARM_LEFT_LEVEL,
            ARM_RIGHT_LEVEL,
            PIVOT_LEFT,
            PIVOT_VERTICAL,
            PIVOT_RIGHT,
            WRIST_LEFT,
            WRIST_STRAIGHT,
            WRIST_RIGHT,
            CLAW_CLOSE,
            CLAW_NEUTRAL,
            CLAW_OPEN,
            OUTTAKE_LOCK_LOCK,
            OUTTAKE_LOCK_UNLOCK,
            DRONE_HOLD,
            DRONE_RELEASE;
}
