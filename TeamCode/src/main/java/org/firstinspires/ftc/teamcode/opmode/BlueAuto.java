package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.BlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.Camera;
import org.firstinspires.ftc.teamcode.system.LoopTime;

@Autonomous
public class BlueAuto extends LinearOpMode {
    BlueTeamPropDetectorPipeline teamPropPipeline = new BlueTeamPropDetectorPipeline();
    Camera camera = new Camera();
    LoopTime loopTime; // custom loop time class

    @Override
    public void runOpMode() {
        camera.initBlueWebcam(hardwareMap);
        BlueTeamPropDetectorPipeline.teamPropPosition position = teamPropPipeline.getPosition();

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
