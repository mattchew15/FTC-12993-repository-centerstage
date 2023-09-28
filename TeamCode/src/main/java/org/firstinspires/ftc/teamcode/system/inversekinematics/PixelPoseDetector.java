package org.firstinspires.ftc.teamcode.system.inversekinematics;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class PixelPoseDetector {
    private final VisionHardware hardware = new VisionHardware();
    private boolean targetFound = false;
    private AprilTagDetection desiredTag = null;

    public int desiredAprilTagId() {
        if (BLUE_AUTO) {
            if (BLUE_POSITION == BLUE_LEFT) {
                return 1;
            } else if (BLUE_POSITION == BLUE_CENTER) {
                return 2;
            } else if (BLUE_POSITION == BLUE_RIGHT) {
                return 3;
            }
        } else if (RED_AUTO) {
            if (RED_POSITION == RED_LEFT) {
                return 4;
            } else if (RED_POSITION == RED_CENTER) {
                return 5;
            } else if (RED_POSITION == RED_RIGHT) {
                return 6;
            }
        }
        return 0;
    }

    public void getPixelPose(Telemetry telemetry, AprilTagProcessor aprilTag) {
        telemetry.addData("Desired apriltag id", desiredAprilTagId());
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == desiredAprilTagId())) {
                targetFound = true;
                desiredTag = detection;
                break;
            } else {
                targetFound = false;
            }
        }

        if (targetFound) {
            APRILTAG_POSE_X = desiredTag.ftcPose.x;
            APRILTAG_POSE_Y = desiredTag.ftcPose.y;
            APRILTAG_POSE_Z = desiredTag.ftcPose.z;

            PIXEL_POSE_X = APRILTAG_POSE_X + PIXEL_DISTANCE_X;
            PIXEL_POSE_Y = APRILTAG_POSE_Y + PIXEL_DISTANCE_Y;
            PIXEL_POSE_Z = APRILTAG_POSE_Z + PIXEL_DISTANCE_Z;

            telemetry.addData("AprilTag x", APRILTAG_POSE_X);
            telemetry.addData("AprilTag y", APRILTAG_POSE_Y);
            telemetry.addData("AprilTag z", APRILTAG_POSE_Z);

            telemetry.addData("Pixel x", PIXEL_POSE_X);
            telemetry.addData("Pixel y", PIXEL_POSE_Y);
            telemetry.addData("Pixel z", PIXEL_POSE_Z);
        } else {
            telemetry.addLine("Target not found");
        }
    }
}
