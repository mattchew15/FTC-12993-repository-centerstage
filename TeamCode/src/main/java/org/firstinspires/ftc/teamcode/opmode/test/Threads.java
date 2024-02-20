package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.system.accessory.ImuThread;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;

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
        ImuThread imuThread = new ImuThread(hardwareMap);
        imuThread.initImuThread();
        imuThread.startThread(this);

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
            telemetry.addData("Imu", imuThread.getImuAngle());
            //telemetry.addData("Imu", AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
            loopTime.updateLoopTime(telemetry);
            telemetry.update();
        }



    }
}
