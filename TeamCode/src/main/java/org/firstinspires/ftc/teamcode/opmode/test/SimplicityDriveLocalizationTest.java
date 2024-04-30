package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveKF;

@Disabled
@Config
@TeleOp(name = "LocalizationTestKF", group = "TestR")
public class SimplicityDriveLocalizationTest extends LinearOpMode
{
    SampleMecanumDriveKF drive;

    public static double startCov1 = 0.01, startCov2 = 0.01, startCov3 = 0.01;
    public static double gainCov1 = 0.1, gainCov2 = 0.1, gainCov3 = 0.1;
    public static double processNoise1 = 0.1, processNoise2 = 0.1, processNoise3 = 0.1;
    public static double sensorNoise1 = 0.1, sensorNoise2 = 0.1, sensorNoise3 = 0.1;
    public static boolean setH = true;
    @Override
    public void runOpMode() throws InterruptedException
    {
        drive = new SampleMecanumDriveKF(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveSetUp();
        waitForStart();

        while (!isStopRequested())
        {
            driveSetUp();
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            Pose2d poseKalman = drive.getPoseKF(poseEstimate); // Pass odometry as obs to check the math
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("KF: X", poseKalman.getX());
            telemetry.addData("KF: Y", poseKalman.getY());
            telemetry.addData("KF: heading", poseKalman.getHeading());
            telemetry.update();
        }

    }
    private void driveSetUp()
    {
        drive.kalman.setH(setH); // Using camera, if distance sensor false
        drive.kalman.setCovarianceGain(gainCov1, gainCov2, gainCov3);
        drive.kalman.setProcessNoise(processNoise1, processNoise2, processNoise3);
        drive.kalman.setSensorNoise(sensorNoise1, sensorNoise2, sensorNoise3);
        drive.kalman.setSTARTING_COVARIANCE(startCov1, startCov2, startCov3);
    }

}
