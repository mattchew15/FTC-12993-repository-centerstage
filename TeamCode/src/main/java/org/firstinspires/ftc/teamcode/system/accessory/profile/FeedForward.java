package org.firstinspires.ftc.teamcode.system.accessory.profile;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.accessory.PID;

/** This will handle the output if you want to use a motion profile,
 * To initialize in init: PID, Tolerance, MaxOutput
 * To loop: update target, create a profile, read, and output
 */
public class FeedForward
{


    public enum FeedForwardMode
    {
        NULL,
        CONSTANT,
        ANGLE_COS,
        ANGLE_SIN
    }

    ElapsedTime timer;
    private ProfileState state;
    private double minFeedForward = 0.0;
    private double maxFeedForward = 0.0;
    private double currentFeedForward = 0.0;
    private double tolerance = 0.0;
    private double pow = 0.0;
    private double position = 0.0;
    private double target = 0.0;
    private double maxOutput = 1.0;
    private PID pid;
    private AsymmetricMotionProfile profile;
    private Telemetry telemetry;
    private FeedForwardMode mode = FeedForwardMode.NULL;
    private boolean reached = false;

    /**
     * Use this constructor at the final code
     * @param mode The mode of the feedforward
     * @param min The min feedforward value
     * @param max The max feedforward value when you update the feedforward, this doesn't cap the current feedforward
     */
    public FeedForward(FeedForwardMode mode, double min, double max)
    {
        this.mode = mode;
        this.minFeedForward = min;
        this.maxFeedForward = max;
        this.currentFeedForward = minFeedForward;
    }
    public FeedForward(FeedForwardMode mode, double min, double max, Telemetry telemetry)
    {
        this.mode = mode;
        this.minFeedForward = min;
        this.maxFeedForward = max;
        this.currentFeedForward = minFeedForward;
        this.telemetry = telemetry;
    }

    /** This does calculations using the feedforward, this does not take any parameter but you
     * need to call reads() before hand, and updateTarget()*/
    public double calculate()
    { // should return the output after using the profile and the pid and the feedforward
        double targetOffSet = target - position;
        if (timer == null)
        {
            timer = new ElapsedTime();
            if (telemetry != null)
            {
                telemetry.addData("Time", "Initiated");
            }
        }
        if (profile != null)
        {
            this.state = profile.calculate(timer.time());
            this.target = state.x + (targetOffSet);
        }
        if (pid != null)
        {
            this.pow = calculatePID(target + targetOffSet, position);
            switch (mode)
            {
                case CONSTANT:
                    this.pow += currentFeedForward * Math.signum(targetOffSet);
                    break;
                case ANGLE_COS:
                    this.pow += currentFeedForward * Math.cos(position);
                    break;
                case ANGLE_SIN:
                    this.pow += currentFeedForward * Math.sin(position);
                    break;
                default:
            }
            this.pow = MathUtils.clamp(pow, -1, 1);
        }
        this.reached = Math.abs((target + targetOffSet) - position) < tolerance;
        if (telemetry != null)
        {
            telemetry.addData("Time at calc", timer.time());
            telemetry.update();
        }
        return pow;
    }

    public void setPID(double kp, double ki, double kd)
    {
        this.pid = new PID(kp, ki, kd, 0,0);
    }
    private double calculatePID(double target, double position)
    {
        return pid.update(target, position, maxOutput);
    }
    public void setFeedforward(double min)
    {
        this.minFeedForward = min;
        currentFeedForward = minFeedForward;
    }
    public void updateMode(FeedForwardMode mode)
    {
        this.mode = mode;
    }

    public void setProfile(AsymmetricMotionProfile profile)
    {
        this.profile = profile;
    }
    public void setMaxOutput(double max)
    {
        this.maxOutput = max;
    }
    public boolean hasReached()
    {
        return reached;
    }
    public void setPos(double pos)
    {
        this.position = pos;
    }
    public void setTolerance(double tolerance)
    {
        this.tolerance = tolerance;
    }
    public void setTarget(double target)
    {
        this.target = target;
    }
    public void updateFeedforward(double percentage)
    {
        this.currentFeedForward = minFeedForward + (maxFeedForward - minFeedForward) * percentage;
    }
    public void resetTime()
    {
        if (timer != null)
        {
            timer.reset();
        }
    }
    /** Loop reads before calculating the output */
    public void reads(double pos, AsymmetricMotionProfile profile)
    {
        setProfile(profile);
        setPos(pos);
    }

    /** Use this to set the new target and also reset the timer */
    public void updateTarget(double target)
    {
        setTarget(target);
        resetTime();
    }

}
