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

    //Intake slides motor
    public static double
            EXTENSION_TICKS_PER_REVOLUTION = 146.44,
            EXTENSION_TICKS_PER_DEGREES = EXTENSION_TICKS_PER_REVOLUTION / 360,
            EXTENSION_SPOOL_RADIUS_CM = 2.15,
            EXTENSION_TICKS_PER_CM = EXTENSION_TICKS_PER_REVOLUTION / (2 * Math.PI * EXTENSION_SPOOL_RADIUS_CM);

    // Outtake lift Motor
    public static double
            LIFT_TICKS_PER_REVOLUTION = 103.8,
            LIFT_TICKS_PER_DEGREES = LIFT_TICKS_PER_REVOLUTION / 360, // Ticks per revolution / 360, 36
            LIFT_SPOOL_RADIUS_CM = 1.8,
            LIFT_TICKS_PER_CM = LIFT_TICKS_PER_REVOLUTION / (2 * Math.PI * LIFT_SPOOL_RADIUS_CM);

    // Pitch
    public static double
            PITCH_TICKS_PER_REVOLUTION = 3895.9,
            PITCH_TICKS_PER_DEGREES = PITCH_TICKS_PER_REVOLUTION / 360;

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
