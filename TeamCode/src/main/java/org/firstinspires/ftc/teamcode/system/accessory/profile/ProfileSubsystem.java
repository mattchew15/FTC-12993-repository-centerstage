package org.firstinspires.ftc.teamcode.system.accessory.profile;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.accessory.FullStateFeedback;
import org.firstinspires.ftc.teamcode.system.accessory.PID;


@Config
public class ProfileSubsystem
{
    public enum SubsystemMode
    {
        NULL,
        CONSTANT,
        ANGLE_COS,
        ANGLE_SIN,
        FullState
    }

    private ElapsedTime timer;
    public ProfileState state;
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
    private SubsystemMode mode = SubsystemMode.NULL;
    private boolean reached = false;
    private FullStateFeedback fullStateFeedback = new FullStateFeedback(0.00001, 0.00004);

    /**
     * Use this constructor at the final code
     *
     * @param mode The mode of the feedforward
     * @param min  The min feedforward value
     * @param max  The max feedforward value when you update the feedforward, this doesn't cap the current feedforward
     */
    public ProfileSubsystem(SubsystemMode mode, double min, double max)
    {
        this.mode = mode;
        this.minFeedForward = min;
        this.maxFeedForward = max;
        this.currentFeedForward = minFeedForward;
    }

    public ProfileSubsystem(SubsystemMode mode)
    {
        this.mode = mode;
    }

    public ProfileSubsystem()
    {

    }
    public ProfileSubsystem(PID pid)
    {
        this.pid = pid;
    }

    public ProfileSubsystem(SubsystemMode mode, double min, double max, Telemetry telemetry)
    {
        this.mode = mode;
        this.minFeedForward = min;
        this.maxFeedForward = max;
        this.currentFeedForward = minFeedForward;
        this.telemetry = telemetry;
    }
    public double calculate()
    {
        // should return the output after using the profile and the pid and the feedforward
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
                case FullState:
                    this.pow += fullStateFeedback.updateWithError((target + targetOffSet) - position, position, profile.state.v) * Math.signum(targetOffSet);
                    break;
                default:

            }
            this.pow = MathUtils.clamp(pow, -1, 1);
        }
        this.reached = Math.abs((target + targetOffSet) - position) < tolerance;
        return pow;
    }
/*
    // I could probably merge with the feedforward
    public double calculateFullState()
    {
        // should return the output after using the profile and the pid and the feedforward
        // Is the targetOffSet
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
            // Ngl this might work or not, it does...
            this.pow = calculatePID(target + targetOffSet, position);
            this.pow += fullStateFeedback.updateWithError((target + targetOffSet) - position, position, state.v) * Math.signum(targetOffSet);
            this.pow = MathUtils.clamp(pow, -1, 1);
        }
        this.reached = (Math.abs((target + targetOffSet) - position) < tolerance);
        return pow;
    }
    */

    public void setPID(double kp, double ki, double kd)
    {
        this.pid = new PID(kp, ki, kd, 0, 0);
    }

    public void setPID(double kp, double ki, double kd, double integralSumLimit)
    {
        this.pid = new PID(kp, ki, kd, integralSumLimit, 0);
    }

    // Don't really do this as the kF makes the profile overshoot
    public void setPID(double kp, double ki, double kd, double integralSumLimit, double kf)
    {
        this.pid = new PID(kp, ki, kd, integralSumLimit, kf);
    }

    private double calculatePID(double target, double position)
    {
        return pid.update(target, position, maxOutput);
    }

    public void setFeedforward(double min, SubsystemMode mode)
    {
        this.mode = mode;
        this.minFeedForward = min;
        currentFeedForward = minFeedForward;
    }

    public void setMode(SubsystemMode mode)
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
    public double getTime()
    {
        return timer.time();
    }

    public void setFullStateFeedback(double kPos, double kVel)
    {
        this.fullStateFeedback = new FullStateFeedback(kPos, kVel);
    }

    // maybe now it will work, if it does, just call this after calculate
    public double getX()
    {
        //The bottom one works without having to call calculate() one more time: return profile.calculate(timer.time()).x;
        return profile.state.x;
    }
    public double getV()
    {
        return profile.state.v;
    }
    public double getA()
    {
        return profile.state.a;
    }
    public void initState()
    {
        state = new ProfileState(0, 0, 0);
    }

}
