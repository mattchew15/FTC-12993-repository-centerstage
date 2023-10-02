package org.firstinspires.ftc.teamcode.opmode.test.inversekinematics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.accessory.Print;
import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;
import org.firstinspires.ftc.teamcode.system.inversekinematics.PixelPoseDetector;
import org.firstinspires.ftc.teamcode.system.vision.BlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.Others.YCrCbRedTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.RedTeamPropDetectorPipeline;

@TeleOp(group = "test")
public class RedAprilTagPoseDetection extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Print print = new Print();
    private final PixelPoseDetector detector = new PixelPoseDetector();

    @Override
    public void runOpMode() {
        SetAuto.setRedAuto();
        hardware.initWebcam(hardwareMap);

        while (!isStarted()) {
            print.telemetryRedWebcam(hardware.getRedPosition());
            telemetry.update();
        }
        RedTeamPropDetectorPipeline.TeamPropPosition position = hardware.getRedPosition();

        hardware.initAprilTag(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            print.telemetryRedWebcam(hardware.getRedPosition());
            detector.getPixelPose(telemetry, hardware.getAprilTag());
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
