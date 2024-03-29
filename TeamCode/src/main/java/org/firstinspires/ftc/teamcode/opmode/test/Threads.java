package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.system.accessory.imu.ImuCompletableFuture;
import org.firstinspires.ftc.teamcode.system.accessory.imu.ImuFuture;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;

import java.util.concurrent.ExecutionException;

@Autonomous(name="Threads test")
public class Threads extends LinearOpMode
{

    LoopTime loopTime = new LoopTime();
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException
    {

        FtcDashboard dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        //ImuThread imuThread = new ImuThread(hardwareMap);
        //imuThread.initImuThread();
        //imuThread.startThread(this);
        //ImuFuture imuFuture = new ImuFuture();
        //imuFuture.initImu(hardwareMap);


        //imuCompletableFuture.run();


        /*
        FtcDashboard dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

         */
        waitForStart();

        while (!isStopRequested() && opModeIsActive())
        {


        }
    }
}
