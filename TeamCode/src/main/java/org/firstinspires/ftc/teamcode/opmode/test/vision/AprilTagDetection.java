package org.firstinspires.ftc.teamcode.opmode.test.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.accessory.Print;
import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;

@Autonomous(group = "test")
public class AprilTagDetection extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Print print = new Print();

    @Override
    public void runOpMode() {
        hardware.initAprilTag(hardwareMap);

        while(!isStarted()) {
            print.telemetryAprilTag(hardware.getAprilTag());
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            print.telemetryAprilTag(hardware.getAprilTag());
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
