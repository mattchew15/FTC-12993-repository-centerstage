package org.firstinspires.ftc.teamcode.system.accessory.imu;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicReference;

@Deprecated
public class ImuFuture
{
    HardwareMap hardwareMap;
    IMU imu;
    AtomicReference<Double> imuAngle;
    private final ExecutorService service = Executors.newSingleThreadExecutor();
    private Future<AtomicReference<Double>> future;

    public void initImu(HardwareMap hardwareMap)
    {
        this.hardwareMap = hardwareMap;
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));


    }

    public void start() throws ExecutionException, InterruptedException
    {

        future =  service.submit(
                () -> new AtomicReference<>(AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)))
        );
        future.get();
    }

    public double get() throws ExecutionException, InterruptedException
    {
        // this might generate an infinite loop
            if (future.isDone())
            {
                imuAngle = future.get();
            }

        return imuAngle.get();
    }

    public void close()
    {
        service.shutdownNow();
    }


}

