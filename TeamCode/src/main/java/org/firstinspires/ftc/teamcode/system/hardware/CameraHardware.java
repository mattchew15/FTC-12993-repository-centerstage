package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.system.vision.PreloadDetection;
import org.firstinspires.ftc.teamcode.system.vision.RelocalizationAprilTagPipeline;
import org.firstinspires.ftc.teamcode.system.vision.YCrCbBlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.YCrCbRedTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.visiontest.RailAdjustAprilTag;
import org.firstinspires.ftc.teamcode.system.visiontest.RelocalizationAprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class CameraHardware {

    private OpenCvWebcam sideWebcam, backWebcam;
    private final YCrCbBlueTeamPropDetectorPipeline  bluePipeline = new YCrCbBlueTeamPropDetectorPipeline();
    private final YCrCbRedTeamPropDetectorPipeline redPipeline = new YCrCbRedTeamPropDetectorPipeline();
    private PreloadDetection preloadPipeline;
    private RelocalizationAprilTagPipeline relocalizationPipeline;
    private RailAdjustAprilTag railAdjustPipeline;
    private AprilTagProcessor aprilTag;

    public void initWebcam(HardwareMap hwMap) {
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

    public void initBackWebcam(HardwareMap hardwareMap, int purplePlacement)
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
        backWebcam.closeCameraDevice();
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

    //Pipeline has to be in the same class as where webcam stuff is initialized
    public AprilTagProcessor getAprilTag() { return aprilTag; }
}
