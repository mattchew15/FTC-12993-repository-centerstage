package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pipelines.RedTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.Camera;

@Autonomous
public class RedAuto extends LinearOpMode {
    RedTeamPropDetectorPipeline teamPropPipeline = new RedTeamPropDetectorPipeline();
    Camera camera = new Camera();
    RedTeamPropDetectorPipeline.teamPropPosition position = teamPropPipeline.getPosition();

    @Override
    public void runOpMode() {
        camera.initRedWebcam(hardwareMap);

        while(!isStarted()) {
            teamPropPipeline.printPosition(telemetry);
            telemetry.update();
        }

        waitForStart();
        while(!isStarted()) {
            teamPropPipeline.printPosition(telemetry);
            telemetry.update();
        }

        //place purple pixel at 'position'
    }
}
