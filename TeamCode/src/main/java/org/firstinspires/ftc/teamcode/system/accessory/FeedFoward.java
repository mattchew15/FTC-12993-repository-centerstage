package org.firstinspires.ftc.teamcode.system.accessory;

import static java.lang.Math.*;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.accessory.profile.ProfileState;

public class FeedFoward
{
    public double KV, KP, KA;
    double kG;
    private double lastVel;
    ElapsedTime timer = new ElapsedTime();
    private boolean reached;

    public enum SubsystemMode
    {
        NULL,
        CONSTANT,
        ANGLE_COS,
        ANGLE_SIN,
        FULL_STATE
    }
    SubsystemMode mode = SubsystemMode.CONSTANT;

    public ProfileState state;
    private double constantGravityFF;
    private double minFeedForward = 0.0;
    private double maxFeedForward = 0.0;
    private double currentFeedForward = 0.0;
    private double tolerance = 0.0;
    private double pow = 0.0;
    private PID pid;
    private Telemetry telemetry;
    public FeedFoward(PID pid)
    {
        this.pid = pid;
    }

    public FeedFoward calculate(double target, double state, double maxOutput, double pitchAngle)
    {
        if (timer == null)
        {
            timer = new ElapsedTime();
        }
        if (pid != null)
        {
            this.pow = pid.update(target,state, maxOutput);
            double error = target - state;
            switch (mode)
            {
                case ANGLE_SIN:
                    this.pow += sin(pitchAngle) * currentFeedForward * signum(error); // as the arm approaches 90º the sin approaches 1
                case ANGLE_COS:
                    this.pow += cos(pitchAngle) * currentFeedForward * signum(error); // as the arm approaches 90º the cos approaches 0
                case CONSTANT:
                    this.pow += currentFeedForward * signum(error);
            }

        }
        return this;
    }

    public double centerOffGravity(double pitchAngle, double state)
    {
        kG = 0.1; // Tune value

        double extension = 0.24; // each mesumi slides length
        double lastLength = 0.35; // Outtake + slider
        double maxDistCOG = 3;
        double lengthForward = 3;

        double m_1 = 0.3; //kg
        double m_2 = 0.3;
        double m_3 = 0.3;
        double m_4 = 0.5;


        double centerOfGravity = ((m_1 * extension) + (m_2 * (extension*2)) + (m_3 * (extension*3)) + (m_4 * (lastLength + extension*3))) / (m_1 + m_2 + m_3 + m_4);
        double lengthOfExtension = Math.hypot(centerOfGravity, state); // might pass this as
        double ratio = lengthOfExtension / maxDistCOG;

        double ff = kG * Math.cos(pitchAngle) * ratio;
        currentFeedForward = ff;
        return pow + ff; // this should be the final value for the motor
    }
}
