package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.hardware.Print;
import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;

@Autonomous
public class ApriltagDetection extends LinearOpMode {
    VisionHardware robot = new VisionHardware();
    Print print = new Print();

    @Override
    public void runOpMode() {
        robot.initApriltag(hardwareMap);

        while(!isStarted()) {
            print.telemetryAprilTag(telemetry);

            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            print.telemetryAprilTag(telemetry);

            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
