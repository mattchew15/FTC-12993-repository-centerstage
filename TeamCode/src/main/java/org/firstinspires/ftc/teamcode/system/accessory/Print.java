package org.firstinspires.ftc.teamcode.system.accessory;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.system.vision.BlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.RedTeamPropDetectorPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class Print {
    private Telemetry telemetry;
    private final RobotHardware hardware = new RobotHardware();
    private final BlueTeamPropDetectorPipeline bluePipeline = new BlueTeamPropDetectorPipeline();
    private final RedTeamPropDetectorPipeline redPipeline = new RedTeamPropDetectorPipeline();

    public void telemetryTeamProp() {
        if (BLUE_AUTO) {
            telemetry.addData("Position", hardware.getBluePosition());
            telemetry.addData("cx", hardware.getBlueCx());
            telemetry.addData("cy", hardware.getBlueCy());
        } else if (RED_AUTO) {
            telemetry.addData("Position", hardware.getRedPosition());
            telemetry.addData("cx", hardware.getBlueCx());
            telemetry.addData("cy", hardware.getBlueCy());
        }
    }

    public void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = hardware.getAprilTag().getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
}
