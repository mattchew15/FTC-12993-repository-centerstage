package org.firstinspires.ftc.teamcode.system.accessory.imu;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicReference;

public class ImuCompletableFuture
{
    HardwareMap hardwareMap;
    IMU imu;
    AtomicReference<Double> imuAngle = new AtomicReference<>(null);

    public void initImu(HardwareMap hardwareMap)
    {
        this.hardwareMap = hardwareMap;
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));


    }

    public CompletableFuture<Double> run()
    {

       return CompletableFuture.supplyAsync(() -> AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
    }

    public double get(LinearOpMode opMode) throws ExecutionException, InterruptedException
    {
        CompletableFuture<Double> future;
        if (imuAngle == null)
        {
             future = run();
             future.get();
             if (future.isDone()) future.thenAccept(result -> imuAngle.set(result));
        }

        return imuAngle.get();
    }
    

}

