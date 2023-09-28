package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

public class SetAuto {
    public static void setRedAuto(){
        RED_AUTO = true;
        BLUE_AUTO = false;
    }
    public static void setBlueAuto(){
        RED_AUTO = false;
        BLUE_AUTO = true;
    }

}
