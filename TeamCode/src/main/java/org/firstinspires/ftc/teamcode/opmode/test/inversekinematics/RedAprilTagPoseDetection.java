package org.firstinspires.ftc.teamcode.opmode.test.inversekinematics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;
import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;
import org.firstinspires.ftc.teamcode.system.inversekinematics.PixelPoseDetector;
import org.firstinspires.ftc.teamcode.system.vision.YCrCbRedTeamPropDetectorPipeline;

@TeleOp(group = "test")
public class RedAprilTagPoseDetection extends LinearOpMode {
    private final VisionHardware hardware = new VisionHardware();
    private final PixelPoseDetector detector = new PixelPoseDetector();

    @Override
    public void runOpMode() {
        SetAuto.setRedAuto();
        hardware.initRedWebcam(hardwareMap);

        while (!isStarted()) {
            hardware.telemetryRedWebcam(telemetry);
            telemetry.update();
        }
        YCrCbRedTeamPropDetectorPipeline.TeamPropPosition position = hardware.getRedPosition();
        Globals.RED_POSITION = position;

        hardware.closeRedWebcam();
        hardware.initApriltag(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            hardware.telemetryRedWebcam(telemetry);
            detector.getPixelPose(telemetry, hardware.getAprilTag());
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
