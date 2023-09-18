package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.LoopTime;
import org.firstinspires.ftc.teamcode.system.RedTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.Camera;

@Autonomous
public class RedAuto extends LinearOpMode {
    RedTeamPropDetectorPipeline teamPropPipeline = new RedTeamPropDetectorPipeline();
    Camera camera = new Camera();
    LoopTime loopTime; // custom loop time class

    @Override
    public void runOpMode() {
        camera.initRedWebcam(hardwareMap);
        RedTeamPropDetectorPipeline.teamPropPosition position = teamPropPipeline.getPosition();

        while(!isStarted()) {
            camera.telemetryBlueWebcam(telemetry);

            telemetry.update();
        }

        waitForStart();
        while(opModeIsActive()) {
            loopTime.updateLoopTime(telemetry);
            camera.telemetryBlueWebcam(telemetry);

            telemetry.update();
        }

        //place purple pixel at 'position'
    }
}
