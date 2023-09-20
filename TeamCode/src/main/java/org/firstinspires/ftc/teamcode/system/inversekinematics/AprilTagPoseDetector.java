package org.firstinspires.ftc.teamcode.system.inversekinematics;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;
import org.firstinspires.ftc.teamcode.system.vision.BlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagPoseDetector {
    private final VisionHardware hardware = new VisionHardware();
    private final AprilTagProcessor aprilTag = hardware.getAprilTag();
    private boolean targetFound = false;
    private AprilTagDetection desiredTag = null;

    public int desiredAprilTagId() {
        if (Globals.BLUE_AUTO) {
            if (hardware.getBluePosition() == BlueTeamPropDetectorPipeline.TeamPropPosition.LEFT) {
                return 1;
            } else if (hardware.getBluePosition() == BlueTeamPropDetectorPipeline.TeamPropPosition.CENTER) {
                return 2;
            } else if (hardware.getBluePosition() == BlueTeamPropDetectorPipeline.TeamPropPosition.RIGHT) {
                return 3;
            }
        } else if (Globals.RED_AUTO) {
            if (hardware.getBluePosition() == BlueTeamPropDetectorPipeline.TeamPropPosition.LEFT) {
                return 4;
            } else if (hardware.getBluePosition() == BlueTeamPropDetectorPipeline.TeamPropPosition.CENTER) {
                return 5;
            } else if (hardware.getBluePosition() == BlueTeamPropDetectorPipeline.TeamPropPosition.RIGHT) {
                return 6;
            }
        }
        return 0;
    }

    public void printId(Telemetry telemetry) {
        telemetry.addData("Desired apriltag id", desiredAprilTagId());
    }

    public void getAprilTagPose(Telemetry telemetry) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == desiredAprilTagId())) {
                targetFound = true;
                desiredTag = detection;
                break;
            }
        }

        if (targetFound) {
            Globals.APRILTAG_POSE_X = desiredTag.ftcPose.x;
            Globals.APRILTAG_POSE_Y = desiredTag.ftcPose.y;
            Globals.APRILTAG_POSE_Z = desiredTag.ftcPose.z;

            telemetry.addData("AprilTag x", Globals.APRILTAG_POSE_X);
            telemetry.addData("AprilTag y", Globals.APRILTAG_POSE_Y);
            telemetry.addData("AprilTag z", Globals.APRILTAG_POSE_Z);
        } else {
            telemetry.addLine("Target not found");
        }
    }
}
