package org.firstinspires.ftc.teamcode.opmode.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.system.hardware.CameraHardware;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.vision.PreloadDetection;
import org.firstinspires.ftc.teamcode.system.vision.RelocalizationAprilTagPipeline;
import org.firstinspires.ftc.teamcode.system.vision.visionProcessorAPI.PreloadDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp(name="Test Preload", group = "Test")
public class TestPreload extends LinearOpMode
{
    DriveBase driveBase = new DriveBase();
    OpenCvCamera backWebcam;
    SampleMecanumDrive drive;
    public static int PURPLE = 0;
    public static int START_X = 0, START_Y = 0, START_H = 0;
    CameraHardware cameraHardware = new CameraHardware();
    @Override
    public void runOpMode() throws InterruptedException
    {
        cameraHardware.initBackWebcamVP(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();
        drive.setPoseEstimate(new Pose2d(START_X, START_Y, START_H));

        waitForStart();

        while (opModeIsActive() && !isStopRequested())
        {
            drive.update();
            driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x);

            if (gamepad1.left_trigger > 0.2)
            {

                telemetry.addData("Detecting pixel", cameraHardware.getPreloadYellowPose());

            }

            //telemetry.addData("X", drive.getPoseEstimate().getX());
            //telemetry.addData("Y", drive.getPoseEstimate().getY());
            //telemetry.addData("H", drive.getPoseEstimate().getHeading());
            telemetry.update();



        }
        backWebcam.closeCameraDevice();



    }
}
