package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;

@Autonomous(group = "test")
public class BlueVision extends LinearOpMode {
    private final VisionHardware hardware = new VisionHardware();

    @Override
    public void runOpMode() {
        hardware.initBlueWebcam(hardwareMap);

        while (!isStarted()) {
            hardware.telemetryBlueWebcam(telemetry);
            telemetry.update();
        }

        waitForStart();
        hardware.stopStreamingBlueWebcam();
        hardware.initApriltag(hardwareMap);

        while (opModeIsActive()) {
            hardware.telemetryAprilTag(telemetry);
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
