package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.system.vision.PreloadDetection;
import org.firstinspires.ftc.teamcode.system.vision.RelocalizationAprilTagPipeline;
import org.firstinspires.ftc.teamcode.system.vision.YCrCbBlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.YCrCbRedTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.visiontest.RailAdjustAprilTag;
import org.firstinspires.ftc.teamcode.system.visiontest.RelocalizationAprilTag;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

public class CameraHardware
{

    private OpenCvWebcam sideWebcam, backWebcam;
    private final YCrCbBlueTeamPropDetectorPipeline  bluePipeline = new YCrCbBlueTeamPropDetectorPipeline();
    private final YCrCbRedTeamPropDetectorPipeline redPipeline = new YCrCbRedTeamPropDetectorPipeline();
    private PreloadDetection preloadPipeline;
    private RelocalizationAprilTagPipeline relocalizationPipeline;
    private RailAdjustAprilTag railAdjustPipeline;
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private AprilTagLibrary library;

    public void initWebcam(HardwareMap hwMap)
    {
        int cameraMonitorViewId;
        cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        sideWebcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Side camera"), cameraMonitorViewId);
        sideWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                if (BLUE_AUTO) {
                    sideWebcam.setPipeline(bluePipeline);
                    sideWebcam.startStreaming(1280, 960, OpenCvCameraRotation.UPSIDE_DOWN);
                } else if (RED_AUTO) {
                    sideWebcam.setPipeline(redPipeline);
                    sideWebcam.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
                }
            }
            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public void initBackWebcamEOCV(HardwareMap hardwareMap, int purplePlacement)
    {
        backWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Back camera"));
        backWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                preloadPipeline = new PreloadDetection();
                relocalizationPipeline = new RelocalizationAprilTagPipeline();
                railAdjustPipeline = new RailAdjustAprilTag();
                backWebcam.setPipeline(preloadPipeline);
                backWebcam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public void closeWebcam() {
        sideWebcam.closeCameraDevice();
        //backWebcam.closeCameraDevice();
    }

    public Pose2d update(double angle)
    {
        return relocalizationPipeline.getPos(angle);
    }

    public void setPreloadPipeline()
    {
        backWebcam.setPipeline(preloadPipeline);
    }

    public Place getYellowPixelState()
    {
        return preloadPipeline.getYellowPixelState();
    }

    public void setRailAdjustPipeline()
    {
        backWebcam.setPipeline(railAdjustPipeline);
    }

    public void setRelocalizationPipeline()
    {
        backWebcam.setPipeline(relocalizationPipeline);
    }


    /*
    public void initAprilTag(HardwareMap hwMap) {
        if (BLUE_AUTO) {
            blueWebcam.closeCameraDevice();
        } else if (RED_AUTO) {
            redWebcam.closeCameraDevice();
        }

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hwMap.get(WebcamName.class, "AprilTagWebcam))
                .setCameraResolution(new Size(1280, 960))
                .enableLiveView(false)
                .build();
    }

     */

    public void initBackWebcamVP(HardwareMap hardwareMap)
    {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.

                // fx = 549.651, fy = 549.651, cx = 317.108, cy = 236.644, mason like saving the macOs users
                // our own calibrated values: Fx: 904.487 Fy: 904.487 Cx: 643.51 Cy: 361.189
                // new camera calibration Focals (pixels) - Fx: 905.033 Fy: 905.033 Optical center - Cx: 642.569 Cy: 359.582
                .setLensIntrinsics(905.033, 905.033, 642.569, 359.582)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Back camera"));

        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);
        builder.enableLiveView(true);

        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG); // higher fps

        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        library = getCenterStageTagLibrary();


    }
    public void telemetryAprilTag(Telemetry telemetry)
    {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y * 1.3291139241, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }

    public void pauseBackWebcam()
    {
        visionPortal.stopStreaming();
    }
    public void resumeBackWebcam()
    {
        visionPortal.resumeStreaming();
    }
    public Pose2d getNewPose(Pose2d pose, Telemetry telemetry)
    {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null)
            {
                telemetry.addData("Tag id", detection.id);
                telemetry.addData("Field pos", library.lookupTag(detection.id).fieldPosition.get(1));
                double newPose = detection.ftcPose.x + library.lookupTag(detection.id).fieldPosition.get(1);
                return new Pose2d(pose.getX(), newPose, pose.getHeading());

            }
        }
        return pose;
    }
    public Place getPreloadYellowPose(Telemetry telemetry)
    {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null)
            {

            }
        }
        return Place.LEFT;
    }
    public void closeBackWebcam()
    {
        visionPortal.close();
    }


    //Pipeline has to be in the same class as where webcam stuff is initialized
    public AprilTagProcessor getAprilTag() { return aprilTag; }
}
