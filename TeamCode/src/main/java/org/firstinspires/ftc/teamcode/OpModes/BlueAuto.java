package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pipelines.BlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.Camera;

@Autonomous
public class BlueAuto extends LinearOpMode {
    BlueTeamPropDetectorPipeline teamPropPipeline = new BlueTeamPropDetectorPipeline();
    Camera camera = new Camera();

    @Override
    public void runOpMode() {
        camera.initBlueWebcam(hardwareMap);
        BlueTeamPropDetectorPipeline.teamPropPosition position = null;

        while(!isStarted()) {
            position = teamPropPipeline.getPosition();
            telemetry.addData("Team prop", position);
            telemetry.update();
        }

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Team prop", position);
            telemetry.update();
        }

        //place purple pixel at 'position'
    }
}
