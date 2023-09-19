package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.hardware.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.hardware.Print;
import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;

@Autonomous
public class AprilTagTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    Print print = new Print();
    LoopTime loopTime; // custom loop time class

    @Override
    public void runOpMode() {
        Globals.auto = true;

        robot.init(hardwareMap);

        while(!isStarted()) {
            print.telemetryAprilTag(telemetry);

            telemetry.update();
        }
        waitForStart();

        while(opModeIsActive()) { // Main loop
            loopTime.updateLoopTime(telemetry);
            print.telemetryAprilTag(telemetry);

            telemetry.update();

        }
    }
}
