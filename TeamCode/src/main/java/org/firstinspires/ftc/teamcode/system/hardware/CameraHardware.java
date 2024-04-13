package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.system.accessory.AprilTagLocalisation;
import org.firstinspires.ftc.teamcode.system.vision.DashBoardProcessor;
import org.firstinspires.ftc.teamcode.system.vision.visionProcessorAPI.PreloadDetectionPipeline;
import org.firstinspires.ftc.teamcode.system.vision.visionProcessorAPI.RailExtensionPipelineTemp;
import org.firstinspires.ftc.teamcode.system.vision.RelocalizationAprilTagPipeline;
import org.firstinspires.ftc.teamcode.system.vision.YCrCbBlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.YCrCbRedTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.visiontest.RailAdjustAprilTag;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class CameraHardware
{

    private OpenCvWebcam sideWebcam, backWebcam;
    private YCrCbBlueTeamPropDetectorPipeline  bluePipeline;
    private YCrCbRedTeamPropDetectorPipeline redPipeline;
    private AprilTagProcessor aprilTag;
    private PreloadDetectionPipeline preloadDetection;
    private RailExtensionPipelineTemp railExtension;
    private DashBoardProcessor dashBoardProcessor;
    private List<Pose2d> poses = new ArrayList<>();
    private final double CAMERA_OFF_SET = 6.5;
    private AprilTagLocalisation ATLocalisation;
    public Pose2d newPose;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private AprilTagLibrary library;
    private int tagWeSee;
    private int targetTag;

    /*public void initWebcam(HardwareMap hwMap, Telemetry telemetry)
    {
        bluePipeline = new YCrCbBlueTeamPropDetectorPipeline(telemetry);
        redPipeline = new YCrCbRedTeamPropDetectorPipeline(telemetry);
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
    }*/
    public void initWebcam(HardwareMap hwMap, Telemetry telemetry)
    {
        if (BLUE_AUTO)
        {
            bluePipeline = new YCrCbBlueTeamPropDetectorPipeline(telemetry);
            int cameraMonitorViewId;
            cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            sideWebcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam Blue"), cameraMonitorViewId);
            sideWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    sideWebcam.setPipeline(bluePipeline);
                    sideWebcam.startStreaming(1280, 960, OpenCvCameraRotation.UPSIDE_DOWN);
                }
                @Override
                public void onError(int errorCode) {
                }
            });

        }
        else
        {
            redPipeline = new YCrCbRedTeamPropDetectorPipeline(telemetry);
            int cameraMonitorViewId;
            cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            sideWebcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam Red"), cameraMonitorViewId);
            sideWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    sideWebcam.setPipeline(redPipeline);
                    sideWebcam.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
                }
                @Override
                public void onError(int errorCode) {
                }
            });

        }

    }


/*    public void initBackWebcamEOCV(HardwareMap hardwareMap, int purplePlacement)
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
    }*/

    public void closeWebcam() {
        sideWebcam.closeCameraDevice();
        //backWebcam.closeCameraDevice();
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

    public void initBackWebcamVP(HardwareMap hardwareMap, Telemetry telemetry)
    {
        ATLocalisation = new AprilTagLocalisation(telemetry);
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
        preloadDetection = new PreloadDetectionPipeline(aprilTag, telemetry);
        railExtension = new RailExtensionPipelineTemp(aprilTag, telemetry);
        dashBoardProcessor = new DashBoardProcessor();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Back camera"));

        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);
        builder.enableLiveView(false);
        /*builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG); // higher fps

        builder.setAutoStopLiveView(false);*/

        // Set and enable the processor.
        builder.addProcessor(aprilTag);
        builder.addProcessor(preloadDetection);
        //builder.addProcessor(dashBoardProcessor);
        builder.addProcessor(railExtension);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        //visionPortal.stopLiveView(); // idk this should work ig tbh why does it work like this i am literally setting it to false and this sh
        //FtcDashboard.getInstance().startCameraStream(dashBoardProcessor, 0);
        library = getCenterStageTagLibrary();
        visionPortal.setProcessorEnabled(preloadDetection, true);
        visionPortal.setProcessorEnabled(railExtension, true);



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

    /*public Boolean getNewPose(Pose2d pose, Telemetry telemetry)
    {

        for (int i = 0; i <3; i++)
        {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            //telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections)
            {
                if (detection.id == 5)
                {
                    if (detection.metadata != null)
                    {
                        //telemetry.addData("Tag id", detection.id);
                        //telemetry.addData("Field pos", library.lookupTag(detection.id).fieldPosition.get(1));
//              this is y relative to the april tag                     this is x relative to the field/pose
                        double newX = library.lookupTag(detection.id).fieldPosition.get(0) - detection.ftcPose.y / 1.3285651049;


//              this is x relative to the april tag                     this is y relative to the field/pose
                        double newY = detection.ftcPose.x + library.lookupTag(detection.id).fieldPosition.get(1);

                        //Pose2d newPoses = ATLocalisation.getCorrectedPose(pose.getHeading(), detection.ftcPose.x, detection.ftcPose.y / 1.3285651049);
                        //telemetry.addData("Corrected pose", newPoses.toString());
                        telemetry.addData("Non corrected X", newX);
                        telemetry.addData("Non corrected Y", newY);
                        //telemetry.addData("Non corrected pose", String.format("X,Y", newX, newY));
                        poses.add(new Pose2d(newX, newY));
                    }
                }
            }
        }
        if (poses.size() >= 3)
        {
            double x, y;
            // we only need like 3 frames so this should be fine
            x = ((poses.get(0).getX() + poses.get(1).getX() + poses.get(2).getX()) / 3) - 5.76;
            y = (poses.get(0).getY() + poses.get(1).getY() + poses.get(2).getY()) / 3;
            newPose = new Pose2d(x, y, pose.getHeading()); // this should be an average pose
            poses.clear();
            return true;
        }
        return false;
    }*/

    public Boolean getNewPose2(Pose2d pose, int lane, Telemetry telemetry)
    {

        double heading = Math.toDegrees(angleWrap(pose.getHeading() + Math.PI)) * -1;
        for (int i = 0; i <3; i++)
        {
            if (poses.size() >= 3) poses.clear();
            List<AprilTagDetection> detections = aprilTag.getDetections();
            //telemetry.addData("# AprilTags Detected", currentDetections.size());
            tagWeSee = 0;
            targetTag = BLUE_AUTO ? lane : 7 - lane;
            AprilTagDetection detection = null;
            if (detections != null)
            {
                for (AprilTagDetection dect : detections)
                {
                    if (dect.id == targetTag)
                    {
                        tagWeSee = targetTag;
                        detection = dect;
                        break;
                    } else if (dect.id == 2 || dect.id == 5)
                    {
                        tagWeSee = dect.id;
                        detection = dect;
                    } else
                    {
                        tagWeSee = dect.id;
                        detection = dect;
                    }
                }
                if (detection != null)
                {
                    if (detection.metadata != null)
                    {
                        //telemetry.addData("Tag id", detection.id);
                        //telemetry.addData("Field pos", library.lookupTag(detection.id).fieldPosition.get(1));
                        double tagX = detection.ftcPose.x;
                        double tagY = detection.ftcPose.y / 1.3285651049;

                        double robotX = ATLocalisation.xPos(heading, tagX, tagY);
                        double robotY = ATLocalisation.yPos(heading, tagX, tagY);

//              this is y relative to the april tag                     this is x relative to the field/pose
                        double newX = library.lookupTag(detection.id).fieldPosition.get(0) - robotX;

//              this is x relative to the april tag                     this is y relative to the field/pose
                        double newY = library.lookupTag(detection.id).fieldPosition.get(1) + robotY;

                        //Pose2d newPoses = ATLocalisation.getCorrectedPose(pose.getHeading(), detection.ftcPose.x, detection.ftcPose.y / 1.3285651049);
                        //telemetry.addData("Corrected pose", newPoses.toString());
                    /*telemetry.addData("Detected X", tagX);
                    telemetry.addData("Detected Y", tagY);
                    telemetry.addData("Robot X", robotX);
                    telemetry.addData("Robot Y", robotY);
                    telemetry.addData("Corrected X", newX);
                    telemetry.addData("Corrected Y", newY);*/
                        //telemetry.addData("Non corrected pose", String.format("X,Y", newX, newY));
                        poses.add(new Pose2d(newX + (-5.9675), newY + 0));//(3.325))); // heading?
                        //break;
                    }
                }
            }
        }
        if (poses.size() >= 3)
        {
            double x, y;
            // we only need like 3 frames so this should be fine
            x = ((poses.get(0).getX() + poses.get(1).getX() + poses.get(2).getX()) / 3); // - 5.76
            y = (poses.get(0).getY() + poses.get(1).getY() + poses.get(2).getY()) / 3;
            newPose = new Pose2d(x, y, pose.getHeading()); // this should be an average pose
            telemetry.addData("Relocalized x", newPose.getX());
            telemetry.addData("Relocalized y", newPose.getY());
            poses.clear();
            return true;
        }
        return false;
    }

    public Pose2d getNewPose()
    {
        return newPose;
    }

    public Place getPreloadYellowPose()
    {
        return place;
    }
    public double getRailTarget(double headingDegrees, double slideLengthInch, double pitchAngleDegrees)
    {
        double distance = slideLengthInch * Math.cos(Math.toRadians(pitchAngleDegrees)); // inches
        double railAdjustment = (distance - PITCH_OFFSET) * Math.sin(Math.toRadians(headingDegrees)); // inches
        return railExtension.getTarget() + inchesToTicksRailCorrected(railAdjustment); // ticks
//        return railExtension.getTarget(); // ticks??

    }
    public void closeBackWebcam()
    {
        visionPortal.close();
    }


    //Pipeline has to be in the same class as where webcam stuff is initialized
    public AprilTagProcessor getAprilTag() { return aprilTag; }

    public void pauseRailProcessor()
    {
        visionPortal.setProcessorEnabled(railExtension, false);
    }
    public void resumeRailProcessor()
    {
        visionPortal.setProcessorEnabled(railExtension, true);
    }
    public void pausePreloadProcessor()
    {
        visionPortal.setProcessorEnabled(preloadDetection, false);
    }
    public void resumePreloadProcessor()
    {
        visionPortal.setProcessorEnabled(preloadDetection, true);
    }

    public void setDefaultRailPos(double defaultTehe){
        railExtension.setDefault(defaultTehe);
    }
    public double getNumSeenTags()
    {
        return aprilTag.getDetections().size();
    }
    public double getTargetTag()
    {
        return railExtension.getTargetTag();
    }
    public void pauseAprilTagProcessor()
    {
        visionPortal.setProcessorEnabled(aprilTag, false);
    }
    public void resumeAprilTagProcessor()
    {
        visionPortal.setProcessorEnabled(aprilTag, true);
    }
}

