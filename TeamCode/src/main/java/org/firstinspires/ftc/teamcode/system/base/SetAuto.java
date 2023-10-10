package org.firstinspires.ftc.teamcode.system.base;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

public class SetAuto {
    public static void setBlueAuto() {
        BLUE_AUTO = true;
        RED_AUTO = false;
    }

    public static void setRedAuto() {
        BLUE_AUTO = false;
        RED_AUTO = true;
    }

    public static void setFrontAuto() {
        FRONT_AUTO = true;
        BACK_AUTO = false;
    }

    public static void setBackAuto() {
        FRONT_AUTO = false;
        BACK_AUTO = false;
    }
}
