package org.firstinspires.ftc.teamcode.opmode.test.inversekinematics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;
import org.firstinspires.ftc.teamcode.system.inversekinematics.PixelPoseDetector;
import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;
import org.firstinspires.ftc.teamcode.system.vision.YCrCbBlueTeamPropDetectorPipeline;

@TeleOp(group = "test")
public class BlueAprilTagPoseDetection extends LinearOpMode {
    private final VisionHardware hardware = new VisionHardware();
    private final PixelPoseDetector detector = new PixelPoseDetector();

    @Override
    public void runOpMode() {
        SetAuto.setBlueAuto();
        hardware.initBlueWebcam(hardwareMap);

        while (!isStarted()) {
            hardware.telemetryBlueWebcam(telemetry);
            telemetry.update();
        }
        YCrCbBlueTeamPropDetectorPipeline.TeamPropPosition position = hardware.getBluePosition();
        Globals.BLUE_POSITION = position;

        hardware.closeBlueWebcam();
        hardware.initApriltag(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            hardware.telemetryBlueWebcam(telemetry);
            detector.getPixelPose(telemetry, hardware.getAprilTag());
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
