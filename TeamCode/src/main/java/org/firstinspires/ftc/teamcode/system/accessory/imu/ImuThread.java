package org.firstinspires.ftc.teamcode.system.accessory.imu;

import androidx.annotation.GuardedBy;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ImuThread
{
    private final Object lock = new Object();

    HardwareMap hardwareMap;
    @GuardedBy("lock")
    Thread thread;
    IMU imu;
    volatile double imuAngle = 0;
    private double imuVel;

    public ImuThread(HardwareMap hardwareMap)
    {
        this.hardwareMap = hardwareMap;
    }
    public void initImuThread()
    {
        synchronized (lock)
        {
            imu = hardwareMap.get(BHI260IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        }
    }

    public void startThread(LinearOpMode opMode)
    {
        // lambda method run from Thread
        thread = new Thread(() -> {
            while (!opMode.isStopRequested())
            {
                synchronized (lock)
                {
                    imuAngle = AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
                    imuVel = imu.getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate;
                }
            }
        });
        thread.start();
    }
    public void resetYaw()
    {
        imu.resetYaw();
    }
    public double getImuAngle()
    {
            return imuAngle;
    }
    public Double getImuExternalVel()
    {
        return imuVel;
    }
}
