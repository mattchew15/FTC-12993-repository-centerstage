package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.hardware.Print;
import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;

@Autonomous
public class RedTeamPropDetection extends LinearOpMode {
    VisionHardware robot = new VisionHardware();
    Print print = new Print();

    @Override
    public void runOpMode() {
        robot.initRedWebcam(hardwareMap);

        while(!isStarted()) {
            print.telemetryRedWebcam(telemetry);

            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            print.telemetryRedWebcam(telemetry);

            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
