package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;

@Autonomous(group = "test")
public class BlueTeamPropDetection extends LinearOpMode {
    VisionHardware visionHardware = new VisionHardware();

    @Override
    public void runOpMode() {
        visionHardware.initBlueWebcam(hardwareMap);

        while(!isStarted()) {
            visionHardware.telemetryBlueWebcam(telemetry);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            visionHardware.telemetryBlueWebcam(telemetry);
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
