package org.firstinspires.ftc.teamcode.opmode.test.vision;

import static org.firstinspires.ftc.teamcode.system.vision.YCrCbRedTeamPropDetectorPipeline.RED_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.CameraHardware;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

@TeleOp(group = "Vision")
public class RedTeamPropDetection extends LinearOpMode {
    CameraHardware cameraHardware = new CameraHardware();

    @Override
    public void runOpMode() {
        SetAuto.setRedAuto();
        cameraHardware.initWebcam(hardwareMap);

        while(!isStarted()) {
            telemetry.addData("Position", RED_POSITION);
            telemetry.update();
        }

        cameraHardware.closeWebcam();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Position", RED_POSITION);
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
