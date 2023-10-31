package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.accessory.PrintCamera;
import org.firstinspires.ftc.teamcode.system.Sequences.AutoCommand;
import org.firstinspires.ftc.teamcode.system.vision.BlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.RedTeamPropDetectorPipeline;

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

    // Intake Motor PID Gains
    public static double
            EXTENSION_P = 0,
            EXTENSION_I = 0,
            EXTENSION_D = 0,
            EXTENSION_F = 0,
            EXTENSION_TICKS_PER_REVOLUTION = 146.44,
            EXTENSION_TICKS_PER_DEGREES = EXTENSION_TICKS_PER_REVOLUTION / 360,
            EXTENSION_SPOOL_RADIUS_CM = 2.15,
            EXTENSION_TICKS_PER_CM = EXTENSION_TICKS_PER_REVOLUTION / (2 * Math.PI * EXTENSION_SPOOL_RADIUS_CM);

    // Intake Motor Positions In CM
    public static double
            EXTENSION_RETRACT_CM = 0,
            EXTENSION_EXTEND_CM = 0,
            EXTENSION_LEFT_CM = 0,
            EXTENSION_CENTER_CM = 0,
            EXTENSION_RIGHT_CM = 0,
            EXTENSION_RETRACT = EXTENSION_RETRACT_CM * EXTENSION_TICKS_PER_CM,
            EXTENSION_EXTEND = EXTENSION_EXTEND_CM * EXTENSION_TICKS_PER_CM,
            EXTENSION_LEFT = EXTENSION_LEFT_CM * EXTENSION_TICKS_PER_CM,
            EXTENSION_CENTER = EXTENSION_CENTER_CM * EXTENSION_TICKS_PER_CM,
            EXTENSION_RIGHT = EXTENSION_RIGHT_CM * EXTENSION_TICKS_PER_CM;

    // Intake Servo/Motor Power
    public static double
            INTAKE_STOP = 0, // Intake
            INTAKE_INTAKE = 0,
            INTAKE_REVERSE = 0,
            INTAKE_DEPOSIT = 0,
            BOTTOM_ROLLER_STOP = 0, // Bottom roller
            BOTTOM_ROLLER_INTAKE = 0,
            BOTTOM_ROLLER_REVERSE = 0,
            BOTTOM_ROLLER_DEPOSIT = 0;

    // Intake Servo Positions In Degrees
    public static double
            BRUSH_HEIGHT_TOP = 0, // Brush height
            BRUSH_HEIGHT_MIDDLE = 0,
            BRUSH_HEIGHT_BASE = 0,

            FLAP_CLOSE = 0, // Flap
            FLAP_OPEN = 0,

            LOCK_LOCK = 0, // Lock
            LOCK_UNLOCK = 0;

    // Outtake Motor PID Gains
    public static double
            LIFT_P = 0, // Lift
            LIFT_I = 0,
            LIFT_D = 0,
            LIFT_F = 0,
            LIFT_TICKS_PER_REVOLUTION = 103.8,
            LIFT_TICKS_PER_DEGREES = LIFT_TICKS_PER_REVOLUTION / 360, // Ticks per revolution / 360, 36
            LIFT_SPOOL_RADIUS_CM = 1.8,
            LIFT_TICKS_PER_CM = LIFT_TICKS_PER_REVOLUTION / (2 * Math.PI * LIFT_SPOOL_RADIUS_CM);

    public static double
            PITCH_P = 0, // Pitch
            PITCH_I = 0,
            PITCH_D = 0,
            PITCH_F = 0,
            PITCH_TICKS_PER_REVOLUTION = 3895.9,
            PITCH_TICKS_PER_DEGREES = PITCH_TICKS_PER_REVOLUTION / 360;

    // Outtake Motor Positions In CM/Degrees
    public static double
            LIFT_RETRACT_CM = 0, // Lift
            LIFT_EXTEND_CM = 0,
            LIFT_RETRACT = LIFT_RETRACT_CM * LIFT_TICKS_PER_CM,
            LIFT_EXTEND = LIFT_EXTEND_CM * LIFT_TICKS_PER_CM;

    public static double
            PITCH_ANGLE_30_DEGREES = 0, // Pitch
            PITCH_ANGLE_60_DEGREES = 0,
            PITCH_ANGLE_30 = PITCH_ANGLE_30_DEGREES * PITCH_TICKS_PER_DEGREES,
            PITCH_ANGLE_60 = PITCH_ANGLE_60_DEGREES * PITCH_TICKS_PER_DEGREES;

    //Outtake Servo Positions In Degrees
    public static double

            ARM_DOWN = 0, // Arms
            ARM_GRAB = 0,
            ARM_READY = 0,
            ARM_OUT = 0,

            PIVOT_LEFT = 0, // Pivot
            PIVOT_CENTER = 0,
            PIVOT_RIGHT = 0,

            WRIST_LEFT = 0, // Wrist
            WRIST_CENTER = 0,
            WRIST_RIGHT = 0,

            CLAW_CLOSE = 0, // Claw
            CLAW_NEUTRAL = 0,
            CLAW_OPEN = 0,

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
