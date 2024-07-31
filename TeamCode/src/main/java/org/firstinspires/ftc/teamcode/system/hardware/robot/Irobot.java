package org.firstinspires.ftc.teamcode.system.hardware.robot;

public interface Irobot
{
    public static boolean ENABLED = false;
    default void init() {}

    default void initUpdate() {}
    default void atStart() {}
    void update();
    default void emergencyStop() {}

}
