package org.firstinspires.ftc.teamcode.system.hardware;

public class SetAuto {
    public static void setRedAuto(){
        Globals.RED_AUTO = true;
        Globals.BLUE_AUTO = false;
    }
    public static void setBlueAuto(){
        Globals.RED_AUTO = false;
        Globals.BLUE_AUTO = true;
    }

}
