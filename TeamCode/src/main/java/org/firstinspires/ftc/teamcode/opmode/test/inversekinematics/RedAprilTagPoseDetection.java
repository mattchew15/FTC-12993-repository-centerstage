package org.firstinspires.ftc.teamcode.opmode.test.inversekinematics;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.accessory.Print;
import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;
import org.firstinspires.ftc.teamcode.system.inversekinematics.PixelPoseDetector;

@TeleOp(group = "test")
public class RedAprilTagPoseDetection extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Print print = new Print();
    private final PixelPoseDetector detector = new PixelPoseDetector();

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
            detector.getPixelPose();
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
