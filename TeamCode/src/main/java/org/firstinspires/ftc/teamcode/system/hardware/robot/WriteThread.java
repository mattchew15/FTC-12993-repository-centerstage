package org.firstinspires.ftc.teamcode.system.hardware.robot;

import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.IHardware;

import java.util.ArrayList;

public class WriteThread implements Runnable
{
    ArrayList<IHardware> list;
    public WriteThread(ArrayList<IHardware> list)
    {
        this.list = list;
    }
    @Override
    public void run()
    {
        for (IHardware hardware: list)
        {
            hardware.writes();
        }
    }
}
