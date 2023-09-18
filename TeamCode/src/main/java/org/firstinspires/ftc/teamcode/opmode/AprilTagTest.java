package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.Camera;
import org.firstinspires.ftc.teamcode.system.LoopTime;

@Autonomous
public class AprilTagTest extends LinearOpMode {
    Camera camera = new Camera();
    LoopTime loopTime; // custom loop time class

    @Override
    public void runOpMode() {

        camera.initAprilTag(hardwareMap);

        while(!isStarted()) {
            camera.telemetryAprilTag(telemetry);

            telemetry.update();
        }
        waitForStart();

        while(opModeIsActive()) { // Main loop
            loopTime.updateLoopTime(telemetry);
            camera.telemetryAprilTag(telemetry);

            telemetry.update();

        }
    }
}
