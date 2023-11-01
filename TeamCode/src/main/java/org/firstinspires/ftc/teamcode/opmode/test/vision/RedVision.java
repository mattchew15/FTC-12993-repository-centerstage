/*
package org.firstinspires.ftc.teamcode.opmode.test.vision;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

@Autonomous(group = "test")
public class RedVision extends LinearOpMode {
    @Override
    public void runOpMode() {
        SetAuto.setRedAuto();
        hardware.initWebcam(hardwareMap);

        RED_POSITION = hardware.getRedPosition();

        while (!isStarted()) {
            print.telemetryTeamProp();
            telemetry.update();
        }

        hardware.initAprilTag(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            print.telemetryTeamProp();
            print.telemetryAprilTag();
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}

 */
