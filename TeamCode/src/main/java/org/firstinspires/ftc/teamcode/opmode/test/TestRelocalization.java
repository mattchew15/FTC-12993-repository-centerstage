package org.firstinspires.ftc.teamcode.opmode.test;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.system.hardware.CameraHardware;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;


@TeleOp(name="Test Relocalization")
public class TestRelocalization extends LinearOpMode
{
    DriveBase driveBase = new DriveBase();
    CameraHardware cameraHardware = new CameraHardware();
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    public static int PURPLE = 0;
    public static int START_X = 0, START_Y = 0, START_H = 0;
    @Override
    public void runOpMode() throws InterruptedException
    {

        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();
        cameraHardware.initBackWebcam(hardwareMap, PURPLE);
        drive.setPoseEstimate(new Pose2d(START_X, START_Y, START_H));

        waitForStart();

        while (opModeIsActive() && !isStopRequested())
        {
            drive.update();
            driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x);

            if (gamepad1.left_trigger > 0.2)
            {
                drive.setPoseEstimate(cameraHardware.update(drive.getPoseEstimate().getHeading()));
            }

            telemetry.addData("X", drive.getPoseEstimate().getX());
            telemetry.addData("Y", drive.getPoseEstimate().getY());
            telemetry.addData("H", drive.getPoseEstimate().getHeading());
            telemetry.update();



        }



    }
}
