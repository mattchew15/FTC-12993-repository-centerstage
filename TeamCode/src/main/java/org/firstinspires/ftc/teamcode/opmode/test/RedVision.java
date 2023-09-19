package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.hardware.WebcamHardware;

@Autonomous(group = "test")
public class RedVision extends LinearOpMode {
    private final WebcamHardware hardware = new WebcamHardware();

    @Override
    public void runOpMode() {
        Globals.RED_AUTO = true;
        hardware.initWebcam(hardwareMap);

        while(!isStarted()) {
            hardware.telemetryWebcam(telemetry);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            hardware.telemetryWebcam(telemetry);
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
