package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Pipelines.BlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.Pipelines.RedTeamPropDetectorPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

public class Camera {

    AprilTagProcessor aprilTag;

    public void initBlueWebcam(HardwareMap hwMap) {
        BlueTeamPropDetectorPipeline blueTeamPropPipeline = new BlueTeamPropDetectorPipeline();
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get
                (WebcamName.class, "Blue webcam"), cameraMonitorViewId);

        webcam.setPipeline(blueTeamPropPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public void initRedWebcam(HardwareMap hwMap) {
        RedTeamPropDetectorPipeline redTeamPropPipeline = new RedTeamPropDetectorPipeline();
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get
                (WebcamName.class, "Red webcam"), cameraMonitorViewId);

        webcam.setPipeline(redTeamPropPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public void initAprilTag(HardwareMap hwMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .build();

        VisionPortal  visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hwMap.get(WebcamName.class, "Blue webcam"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();
    }

    public void telemetryBlueWebcam(Telemetry telemetry) {

    }

    public void telemetryAprilTag(Telemetry telemetry) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
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
