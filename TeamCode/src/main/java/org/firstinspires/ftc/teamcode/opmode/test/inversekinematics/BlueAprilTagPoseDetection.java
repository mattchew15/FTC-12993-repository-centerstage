package org.firstinspires.ftc.teamcode.opmode.test.inversekinematics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.OuttakeInverseKinematics.AprilTagPoseDetector;
import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;

@Autonomous(group = "test")
public class BlueAprilTagPoseDetection extends LinearOpMode {
    private final VisionHardware hardware = new VisionHardware();
    private final AprilTagPoseDetector detector = new AprilTagPoseDetector();

    @Override
    public void runOpMode() {
        Globals.BLUE_AUTO = true;
        hardware.initBlueWebcam(hardwareMap);

        while (!isStarted()) {
            hardware.telemetryBlueWebcam(telemetry);
            telemetry.update();
        }

        hardware.closeBlueWebcam();
        hardware.initApriltag(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            hardware.telemetryBlueWebcam(telemetry);
            hardware.telemetryAprilTag(telemetry);
            detector.printId(telemetry);
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
