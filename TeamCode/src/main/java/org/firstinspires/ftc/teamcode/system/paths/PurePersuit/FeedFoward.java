package org.firstinspires.ftc.teamcode.system.paths.PurePersuit;

import com.qualcomm.robotcore.util.ElapsedTime;

public class FeedFoward
{
    public double KV, KP, KA;
    private double lastVel;
    ElapsedTime timer = new ElapsedTime();
    public FeedFoward(double KV, double KP, double KA)
    {
        this.KV = KV;
        this.KP = KP;
        this.KA = KA;
    }
    public double calculate(double targetVel, double currentVel)
    {

       double targetAccel = (currentVel - lastVel) / timer.time();

       return KV * targetVel + KA * targetAccel + KP * (targetVel - currentVel);
    }
}
