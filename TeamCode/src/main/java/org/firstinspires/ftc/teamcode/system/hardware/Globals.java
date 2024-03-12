package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import java.util.List;

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

            RED_AUTO = false;

    // Team Prop Location
    public static int teamPropLocation = 1;
    public static LynxModule chub, expHub;
    public static double EPSILON_DELTA = 0.005;

    // Pitch
    public static double
            //PITCH_TICKS_PER_REVOLUTION = 3895.9,
            //PITCH_TICKS_PER_DEGREES = PITCH_TICKS_PER_REVOLUTION / 360,

            TICKS_PER_BAREMOTOR = 28,

            PITCH_TICKS_PER_REVOLUTION = 4096,
            PITCH_TICKS_PER_DEGREES = (PITCH_TICKS_PER_REVOLUTION / 0.8) / 360,


            GAMEPAD_TRIGGER_THRESHOLD = 0.2,

            RAIL_SERVO_POSITION,
            FINE_ADJUST_HEIGHT_INTERVAL = 5,
            A; // angle offset for autonmous

    public static int
            PITCH_OFFSET_VALUE = 319,
            PITCH_DEFAULT_DEGREE_TICKS = 36,
            PITCH_CLIMB_TICKS = 65,
            UPRIGHT_PITCH_TICKS = 56,
            PITCH_LOW_DEGREE_TICKS = 22,
            PITCH_MID_DEGREE_TICKS = 45,

            INTAKE_SLIDE_EXTENDO_TELEOP_FAR = 690,
            INTAKE_SLIDE_EXTENDO_TELEOP_CLOSE = 410,

            LIFT_INCHES_FOR_MAX_EXTENSION = 23, // 28 is the actual max but we need to give some adjustment room

            // Nats shit bro kinda L
            //LIFT_HIGH_POSITION_TICKS = 600,
            //LIFT_MEDIUM_POSITION_TICKS = 400,
            //LIFT_LOW_POSITION_TICKS = 70,

            //LIFT_MEDIUM_POSITION_PITCHING_TICKS = 580,
            //LIFT_LOW_POSITION_PITCHING_TICKS = 690,
            LIFT_HITS_WHILE_PITCHING_THRESHOLD = 3,

            // Worlds constants
            HIGH_BACKDROP_PRESET_INCHES = 26,
            MID_BACKDROP_PRESET_INCHES = 16,
            LOW_BACKDROP_PRESET_INCHES = 9,

            S = 1, // the side multiplier to change sides for autonomous
            LEFT_OR_RIGHT;

    public static double degreestoTicksPitchMotor(double degrees){
        return degrees * PITCH_TICKS_PER_DEGREES;
    }

    public static double ticksToDegreePitchMotor (double ticks){
        return ticks/PITCH_TICKS_PER_DEGREES;
    }

    public static double ticksToInchesSlidesMotor(double ticks){
        return ((1.33858*Math.PI)/(TICKS_PER_BAREMOTOR*5)) * ticks; //34mm is the diameter of the reel
    }

    public static double inchesToTicksSlidesMotor (double inches){
        return ((TICKS_PER_BAREMOTOR*5)/(1.33858*Math.PI)) * inches; //ticks per inches
        // ratio is 60/12 = 5
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
        if (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        else if (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }

    public static double angleWrapDegrees(double degrees) {
        if (degrees > 180) {
            degrees -= 360;
        }
        else if (degrees < -180) {
            degrees += 360;
        }

        // keep in mind that the result is in radians
        return degrees;
    }

    /**
     *
     * @param current current power returned from the pid
     * @param prev previous power set to the motor
     * @param cachingTolerance threshold for the abs difference between the powers
     * @param motor the motor the power should bet set if one of the conditions is true
     * @return the value to be cached as the previous power
     */
    public static double motorCaching(double current, double prev, double cachingTolerance, DcMotor motor) {
        if (
            //Should it be >= or >
            Math.abs(current - prev) > cachingTolerance ||
                    (current == 0.0 && prev != 0.0) ||
                    (current >= 1.0 && !(prev >= 1.0)) ||
                    (current <= -1.0 && !(prev <= -1.0))
        ) {
            prev = current;
            motor.setPower(current);
        }
        return prev;
    }

    /**
     *
     * @param hubs the list of all hubs
     * @return returns a new list where chub is always at index 0
     */
    public List<LynxModule> setHubs(List<LynxModule> hubs)
    {
        if(hubs.get(0).isParent() && LynxConstants.isEmbeddedSerialNumber(hubs.get(0).getSerialNumber())){
            chub = hubs.get(0);
            expHub = hubs.get(1);
        }else{
            chub = hubs.get(1);
            expHub = hubs.get(0);
        }
        hubs.clear();
        hubs.add(chub);
        hubs.add(expHub);
        return hubs;
    }
}
