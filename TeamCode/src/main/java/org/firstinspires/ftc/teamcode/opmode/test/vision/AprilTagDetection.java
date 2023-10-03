package org.firstinspires.ftc.teamcode.opmode.test.vision;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "test")
public class AprilTagDetection extends LinearOpMode {
    @Override
    public void runOpMode() {
        hardware.initAprilTag(hardwareMap);

        while(!isStarted()) {
            print.telemetryAprilTag();
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            print.telemetryAprilTag();
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
