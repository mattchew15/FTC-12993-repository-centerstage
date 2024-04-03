package org.firstinspires.ftc.teamcode.opmode.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.system.hardware.CameraHardware;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.vision.PreloadDetection;
import org.firstinspires.ftc.teamcode.system.vision.RelocalizationAprilTagPipeline;
import org.firstinspires.ftc.teamcode.system.visiontest.RailAdjustAprilTag;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

@Config
@TeleOp(name="Test Rail", group = "Test")
public class TestRelocalization extends LinearOpMode
{
    DriveBase driveBase = new DriveBase();
    //OpenCvCamera backWebcam;
    CameraHardware cameraHardware = new CameraHardware();
    SampleMecanumDrive drive;
    Pose2d poseEstimate;
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    double heading;
    boolean prev;

    public static int START_X = 0, START_Y = 0, START_H = 0;
    // RelocalizationAprilTagPipeline relocalizationAprilTagPipeline = new RelocalizationAprilTagPipeline();
    //RelocationV2Pipeline relocationV2Pipeline = new RelocationV2Pipeline();
    @Override
    public void runOpMode() throws InterruptedException
    {

        cameraHardware.initBackWebcamVP(hardwareMap, telemetry);
        //relocationV2Pipeline = new RelocationV2Pipeline();
        //relocationV2Pipeline.setTelemetry(telemetry);
        //relocalizationAprilTagPipeline.setTelemetry(telemetry);

        outtakeSubsystem.initOuttake(hardwareMap);
        outtakeSubsystem.hardwareSetup();

        drive = new SampleMecanumDrive(hardwareMap);
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
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
        place = Place.MIDDLE;
        waitForStart();

        while (opModeIsActive() && !isStopRequested())
        {
            drive.update();
            driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            poseEstimate = drive.getPoseEstimate();
            heading = Math.toDegrees(outtakeSubsystem.angleWrap(drive.getPoseEstimate().getHeading() + Math.PI)) * -1;

            if (gamepad1.left_trigger > 0.2)
            {
                telemetry.addData("Updating pose", true);

                /*if(cameraHardware.getNewPose2(drive.getPoseEstimate(), 2, telemetry))
                {
                    drive.setPoseEstimate(cameraHardware.getNewPose());
                }*/
                //drive.setPoseEstimate(relocalizationAprilTagPipeline.getPos(drive.getPoseEstimate().getHeading()));
                //drive.setPoseEstimate(cameraHardware.getNewPose(drive.getPoseEstimate(), telemetry));
                // this corrects for the y value using april tags, return the same x and h than the passed ones,
                // assumes the robot is with heading zero to the april tags
                double t = cameraHardware.getRailTarget();
                telemetry.addData("Rail target", t);
                outtakeSubsystem.setOuttakeRailServo(t);

            }

            if (gamepad1. dpad_down)
            {
                outtakeSubsystem.setOuttakeRailServo(0.487);
            }
            if (gamepad1.right_bumper && !prev)
            {
                outtakeSubsystem.setOuttakeRailServo(RAIL_CENTER_POS -= .1);
            }
            if (gamepad1.left_bumper && !prev)
            {
                outtakeSubsystem.setOuttakeRailServo(RAIL_CENTER_POS += .1);
            }

            prev = gamepad1.right_bumper || gamepad1.left_bumper;
            if (gamepad1.dpad_left)
            {
                place = Globals.Place.LEFT;
            } else if (gamepad1.dpad_right)
            {
                place = Place.RIGHT;
            }

            if (gamepad1.x)
            {
                teamPropLocation = 1;
            }
            else if (gamepad1.a)
            {
                teamPropLocation = 2;
            }
            else if (gamepad1.b)
            {
                teamPropLocation = 3;
            }

            telemetry.addData("Teamprop", teamPropLocation);
            telemetry.addData("Case", place);
            telemetry.addLine();
            telemetry.addData("X", poseEstimate.getX());
            telemetry.addData("Y", poseEstimate.getY());
            telemetry.addData("H", heading);



            telemetry.update();

        }
        cameraHardware.closeBackWebcam();

    }
}
