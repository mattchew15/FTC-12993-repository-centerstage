package org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers;

public interface IHardware
{
    public static boolean ENABLED = false;
    void update();
    void reads();
    void writes();
    default void emergencyStop() {}
}
