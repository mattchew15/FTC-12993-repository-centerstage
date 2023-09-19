package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;

@Autonomous(group = "test")
public class RedTeamPropDetection extends LinearOpMode {
    VisionHardware visionHardware = new VisionHardware();

    @Override
    public void runOpMode() {
        visionHardware.initRedWebcam(hardwareMap);

        while(!isStarted()) {
            visionHardware.telemetryRedWebcam(telemetry);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            visionHardware.telemetryRedWebcam(telemetry);
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
