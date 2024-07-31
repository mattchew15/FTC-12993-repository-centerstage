package org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.system.accessory.PID;

public class MotorFoda implements IHardware
{
    private DcMotorEx motor;
    private double power, prevPower;
    private final Object lock = new Object();
    private final double EPSILON_DELTA = 0.005;
    private boolean async = false;
    private double target;
    private double position;

    private PID pid;
    private double p, i, d;
    private boolean write = true;
    public MotorFoda(DcMotorEx motor)
    {
        this.motor = motor;
    }
    public MotorFoda(DcMotorEx motor, double p, double i, double d)
    {
        this.motor = motor;
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public void updatePower()
    {
        if (
                (Math.abs(power - prevPower) > EPSILON_DELTA) ||
                (power == 0.0 && prevPower != 0.0) ||
                (power >= 1.0 && !(prevPower >= 1.0)) ||
                (power <= -1.0 && !(prevPower <= -1.0))
        )
        {
            prevPower = power;
        }
    }

    public void setPower(double power)
    {
        motor.setPower(power);
    }
    public void setPowerAsync(double power)
    {
        synchronized (lock)
        {
            this.power = power;
            updatePower();
        }
    }
    public void runPID()
    {
        pid.update(target, position, 1);
    }
    public void setAsync(boolean async)
    {
        this.async = async;
    }
    public double getPosition()
    {
       return motor.getCurrentPosition();
    }

    public void update()
    {
        position = getPosition();

    }

    @Override
    public void reads()
    {

    }

    @Override
    public void writes()
    {
        if (async)
        {
            setPowerAsync(power);
        }
        else {
            updatePower();
        }
    }
}
