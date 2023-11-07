package org.firstinspires.ftc.teamcode.system.accessory;

import com.qualcomm.robotcore.util.ElapsedTime;

public class FullStateFeedback
{
    /*
    This type of controller will have much better trajectory tracking performance than pid as in addition
    to driving our robot to the target position, it is able to drive our robot at target velocity.
    In combination with motion profiling this is an incredibly powerful tool.
    You can even extend this method by adding additional states.
    Combined with feedforward control you will have an incredibly robust controller.
     */
    ElapsedTime timer = new ElapsedTime();
    double k1,k2;
    double vel, prevPos;
    public FullStateFeedback(double k1, double k2)
    {
        this.k1 = k1; // Positional gain
        this.k2 = k2; // Velocity gain
    }
    public double update(double target, double position, double targetVel)
    {
        vel = (position - prevPos) / timer.seconds();
        double error = target - position;
        double errorVel = targetVel - vel;
        double update = (error * k1) + (errorVel * k2);

        prevPos = position;
        timer.reset();
        return update;

    }

    // Use this if already have the current velocity of the motor
    public double updateWithError(double target, double position, double targetVel, double velocity)
    {
        double error = target - position;
        double errorVel = targetVel - velocity;
        return (error * k1) + (errorVel * k2);
    }

}
