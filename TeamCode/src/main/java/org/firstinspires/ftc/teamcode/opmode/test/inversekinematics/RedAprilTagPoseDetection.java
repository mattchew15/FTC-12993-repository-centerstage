package org.firstinspires.ftc.teamcode.opmode.test.inversekinematics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;
import org.firstinspires.ftc.teamcode.system.inversekinematics.AprilTagPoseDetector;
import org.firstinspires.ftc.teamcode.system.vision.RedTeamPropDetectorPipeline;

@TeleOp(group = "test")
public class RedAprilTagPoseDetection extends LinearOpMode {
    private final VisionHardware hardware = new VisionHardware();
    private final AprilTagPoseDetector detector = new AprilTagPoseDetector();

    @Override
    public void runOpMode() {
        Globals.RED_AUTO = true;
        hardware.initRedWebcam(hardwareMap);

        while (!isStarted()) {
            hardware.telemetryRedWebcam(telemetry);
            telemetry.update();
        }
        RedTeamPropDetectorPipeline.TeamPropPosition position = hardware.getRedPosition();
        Globals.RED_POSITION = position;

        hardware.closeRedWebcam();
        hardware.initApriltag(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            hardware.telemetryRedWebcam(telemetry);
            detector.printId(telemetry);
            telemetry.addData("Position is", position);
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
