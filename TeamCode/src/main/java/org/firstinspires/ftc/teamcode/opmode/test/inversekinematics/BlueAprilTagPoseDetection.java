package org.firstinspires.ftc.teamcode.opmode.test.inversekinematics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.inversekinematics.AprilTagPoseDetector;
import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;
import org.firstinspires.ftc.teamcode.system.vision.BlueTeamPropDetectorPipeline;

@TeleOp(group = "test")
public class BlueAprilTagPoseDetection extends LinearOpMode {
    private final VisionHardware hardware = new VisionHardware();
    private final AprilTagPoseDetector detector = new AprilTagPoseDetector();

    @Override
    public void runOpMode() {
        Globals.BLUE_AUTO = true;
        hardware.initBlueWebcam(hardwareMap);

        while (!isStarted()) {
            hardware.telemetryBlueWebcam(telemetry);
            telemetry.update();
        }
        BlueTeamPropDetectorPipeline.TeamPropPosition position = hardware.getBluePosition();
        Globals.BLUE_POSITION = position;

        hardware.closeBlueWebcam();
        hardware.initApriltag(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            hardware.telemetryBlueWebcam(telemetry);
            detector.printId(telemetry);
            telemetry.addData("Position is", position);
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
