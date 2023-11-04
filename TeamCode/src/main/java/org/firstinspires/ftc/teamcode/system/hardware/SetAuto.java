package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SetAuto {

    //Static methods that we can run at the start of the opmode - we can put our mirror stuff code in here
    public static void setBlueAuto() {
        BLUE_AUTO = true;
        RED_AUTO = false;
        S = -1; // multiply turrets
        A = Math.toRadians(180); // flip rr coordinates
    }

    public static void setRedAuto() {
        BLUE_AUTO = false;
        RED_AUTO = true;
    }
    /*
    public static void setFrontAuto() {
        FRONT_AUTO = true;
        BACK_AUTO = false;
    }

    public static void setBackAuto() {
        FRONT_AUTO = false;
        BACK_AUTO = false;
    }
     */
}
