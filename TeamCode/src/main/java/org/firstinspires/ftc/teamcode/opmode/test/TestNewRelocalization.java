package org.firstinspires.ftc.teamcode.opmode.test;

import static org.firstinspires.ftc.teamcode.system.hardware.SetAuto.setBlueAuto;
import static org.firstinspires.ftc.teamcode.system.hardware.SetAuto.setRedAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveThread;
import org.firstinspires.ftc.teamcode.system.hardware.CameraHardware;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;

@TeleOp(name = "NEW POSE THING", group = "Test")
public class TestNewRelocalization extends LinearOpMode
{

    DriveBase driveBase = new DriveBase(telemetry);
    @Override
    public void runOpMode() throws InterruptedException {
        //setRedAuto();
        setBlueAuto();
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();
        SampleMecanumDriveThread sampleMecanumDrive = new SampleMecanumDriveThread(hardwareMap);
        sampleMecanumDrive.startImuThread(this);
        sampleMecanumDrive.setPoseEstimate(new Pose2d(40, 35 , Math.toRadians(180)));
        CameraHardware cameraHardware = new CameraHardware();
        cameraHardware.initBackWebcamVP(hardwareMap, telemetry);
        cameraHardware.pauseRailProcessor();
        cameraHardware.pausePreloadProcessor();
        waitForStart();
        while (opModeIsActive())
        {

            driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.left_trigger > 0.2)
            {
                if(cameraHardware.getNewPose3(sampleMecanumDrive.getPoseEstimate(), 2, telemetry,true))
                {
                    sampleMecanumDrive.setPoseEstimate(cameraHardware.getNewPose());
                    telemetry.addLine("WE are re-localizing boysss!!!");
                }
            }
            telemetry.addData("ROBOT X", cameraHardware.ROBOT_X);
            telemetry.addData("ROBOT Y", cameraHardware.ROBOT_Y);

            telemetry.addData("Camera X", cameraHardware.cameraX);
            telemetry.addData("Camera Y", cameraHardware.cameraY);

            sampleMecanumDrive.update(); // this should fixthe heading ig
            telemetry.update();
        }
    }
}
