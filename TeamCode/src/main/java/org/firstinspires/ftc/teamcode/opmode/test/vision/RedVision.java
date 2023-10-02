package org.firstinspires.ftc.teamcode.opmode.test.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.accessory.Print;
import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

@Autonomous(group = "test")
public class RedVision extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Print print = new Print();

    @Override
    public void runOpMode() {
        SetAuto.setRedAuto();
        hardware.initWebcam(hardwareMap);

        while (!isStarted()) {
            print.telemetryRedWebcam(hardware.getRedPosition());
            telemetry.update();
        }

        hardware.initAprilTag(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            print.telemetryRedWebcam(hardware.getRedPosition());
            print.telemetryAprilTag(hardware.getAprilTag());
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
