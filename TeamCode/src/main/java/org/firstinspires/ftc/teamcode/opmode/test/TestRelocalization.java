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
import org.firstinspires.ftc.teamcode.system.vision.RelocationV2Pipeline;
import org.firstinspires.ftc.teamcode.system.visiontest.RailAdjustAprilTag;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp(name="Test Relocalization", group = "Test")
public class TestRelocalization extends LinearOpMode
{
    DriveBase driveBase = new DriveBase();
    //OpenCvCamera backWebcam;
    CameraHardware cameraHardware = new CameraHardware();
    SampleMecanumDrive drive;

    public static int START_X = 0, START_Y = 0, START_H = 0;
    // RelocalizationAprilTagPipeline relocalizationAprilTagPipeline = new RelocalizationAprilTagPipeline();
    //RelocationV2Pipeline relocationV2Pipeline = new RelocationV2Pipeline();
    @Override
    public void runOpMode() throws InterruptedException
    {

        cameraHardware.initBackWebcamVP(hardwareMap);
        //relocationV2Pipeline = new RelocationV2Pipeline();
        //relocationV2Pipeline.setTelemetry(telemetry);
        //relocalizationAprilTagPipeline.setTelemetry(telemetry);

        drive = new SampleMecanumDrive(hardwareMap);
        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        backWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Back camera"), cameraMonitorViewId);
        backWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                backWebcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                backWebcam.setPipeline(relocationV2Pipeline);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();
        drive.setPoseEstimate(new Pose2d(START_X, START_Y, START_H));
*/
        waitForStart();

        while (opModeIsActive() && !isStopRequested())
        {
            drive.update();
            driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x);

            if (gamepad1.left_trigger > 0.2)
            {
                telemetry.addData("Updating pose", true);
                //drive.setPoseEstimate(relocalizationAprilTagPipeline.getPos(drive.getPoseEstimate().getHeading()));
                drive.setPoseEstimate(cameraHardware.getNewPose(drive.getPoseEstimate(), telemetry));
                // this corrects for the y value using april tags, return the same x and h than the passed ones,
                // assumes the robot is with heading zero to the april tags
            }

            telemetry.addData("X", drive.getPoseEstimate().getX());
            telemetry.addData("Y", drive.getPoseEstimate().getY());
            telemetry.addData("H", drive.getPoseEstimate().getHeading());
            telemetry.update();



        }
        cameraHardware.closeBackWebcam();



    }
}
