package org.firstinspires.ftc.teamcode.opmode.test.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;

@Autonomous(group = "test")
public class AprilTagDetection extends LinearOpMode {
    private final VisionHardware hardware = new VisionHardware();

    @Override
    public void runOpMode() {
        hardware.initApriltag(hardwareMap);

        while(!isStarted()) {
            hardware.telemetryAprilTag(telemetry);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            hardware.telemetryAprilTag(telemetry);
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
