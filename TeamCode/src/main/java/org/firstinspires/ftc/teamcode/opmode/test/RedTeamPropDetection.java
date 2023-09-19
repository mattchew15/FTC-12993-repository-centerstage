package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;

@Autonomous(group = "test")
public class RedTeamPropDetection extends LinearOpMode {
    private final VisionHardware hardware = new VisionHardware();

    @Override
    public void runOpMode() {
        hardware.initRedWebcam(hardwareMap);

        while(!isStarted()) {
            hardware.telemetryRedWebcam(telemetry);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            hardware.telemetryRedWebcam(telemetry);
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
