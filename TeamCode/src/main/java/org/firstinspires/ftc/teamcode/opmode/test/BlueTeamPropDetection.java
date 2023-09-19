package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.hardware.Print;
import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;

@Autonomous(group = "test")
public class BlueTeamPropDetection extends LinearOpMode {
    VisionHardware robot = new VisionHardware();
    Print print = new Print();

    @Override
    public void runOpMode() {
        robot.initBlueWebcam(hardwareMap);

        while(!isStarted()) {
            print.telemetryBlueWebcam(telemetry);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            print.telemetryBlueWebcam(telemetry);
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
