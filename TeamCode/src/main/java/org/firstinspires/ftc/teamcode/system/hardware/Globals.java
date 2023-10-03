package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.accessory.Print;
import org.firstinspires.ftc.teamcode.system.vision.BlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.RedTeamPropDetectorPipeline;

@Config // Allows dashboard tune
public class Globals {
    public static RobotHardware hardware;
    public static Telemetry telemetry;
    public static Print print;

    // Auto States
    public static boolean
            BLUE_AUTO = false,
            RED_AUTO = false;

    // Intake Motor PID Gains
    public static double
            EXTENSION_P = 0,
            EXTENSION_I = 0,
            EXTENSION_D = 0,
            EXTENSION_F = 0,
            EXTENSION_TICKS_IN_DEGREES = 700 / 180;

    // Intake Motor Positions
    public static int
            EXTENSION_RETRACT = 0,
            EXTENSION_EXTEND = 0;

    //Intake Servo Positions
    public static double
            BOTTOM_ROLLER_5 = 0, // Bottom roller
            BOTTOM_ROLLER_4 = 0,
            BOTTOM_ROLLER_3 = 0,
            BOTTOM_ROLLER_2 = 0,
            BOTTOM_ROLLER_1 = 0,

            BRUSH_HEIGHT_5 = 0, // Brush height
            BRUSH_HEIGHT_4 = 0,
            BRUSH_HEIGHT_3 = 0,
            BRUSH_HEIGHT_2 = 0,
            BRUSH_HEIGHT_1 = 0,

            FLAP_CLOSE = 0, // Flap
            FLAP_OPEN = 0,

            INTAKE_LOCK_LOCK = 0, // Intake lock
            INTAKE_LOCK_UNLOCK = 0;

    // Outtake Motor PID Gains
    public static double
            LIFT_P = 0, // Lift
            LIFT_I = 0,
            LIFT_D = 0,
            LIFT_F = 0,
            LIFT_TICKS_IN_DEGREES = 700 / 180,

            PITCH_P = 0, // Pitch
            PITCH_I = 0,
            PITCH_D = 0,
            PITCH_F = 0,
            PITCH_TICKS_IN_DEGREES = 700 / 180;

    // Outtake Motor Positions
    public static int
            LIFT_RETRACT = 0, // Lift
            LIFT_EXTEND = 0,

            PITCH_DEGREES_30 = 0, // Pitch
            PITCH_DEGREES_60 = 0;

    //Outtake Servo Positions
    public static double
            RAIL_LEFT = 0, // Rail
            RAIL_CENTER = 0,
            RAIL_RIGHT = 0,

            ARM_LEFT_DOWN = 0, // Arm Left
            ARM_LEFT_LIFT = 0,
            ARM_LEFT_UP = 0,
            ARM_LEFT_LEVEL = 0,

            ARM_RIGHT_DOWN = 0, // Arm Right
            ARM_RIGHT_LIFT = 0,
            ARM_RIGHT_UP = 0,
            ARM_RIGHT_LEVEL = 0,

            PIVOT_LEFT = 0, // Pivot
            PIVOT_VERTICAL = 0,
            PIVOT_RIGHT = 0,

            WRIST_LEFT = 0, // Wrist
            WRIST_STRAIGHT = 0,
            WRIST_RIGHT = 0,

            CLAW_CLOSE = 0, // Claw
            CLAW_NEUTRAL = 0,
            CLAW_OPEN = 0,

            OUTTAKE_LOCK_LOCK = 0, // Outtake lock
            OUTTAKE_LOCK_UNLOCK = 0,

            DRONE_HOLD = 0, // Drone
            DRONE_RELEASE = 0;

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

}
