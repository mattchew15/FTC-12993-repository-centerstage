package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.system.vision.YCrCbBlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.YCrCbRedTeamPropDetectorPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class CameraHardware {


    private OpenCvWebcam blueWebcam, redWebcam;
    private final YCrCbBlueTeamPropDetectorPipeline bluePipeline = new YCrCbBlueTeamPropDetectorPipeline();
    private final YCrCbRedTeamPropDetectorPipeline redPipeline = new YCrCbRedTeamPropDetectorPipeline();
    private AprilTagProcessor aprilTag;



    public void initWebcam(HardwareMap hwMap) {
        int cameraMonitorViewId;
        if (BLUE_AUTO) {
            cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            blueWebcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "WebcamRight"), cameraMonitorViewId);
            blueWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    blueWebcam.setPipeline(bluePipeline);
                    blueWebcam.startStreaming(1280, 960, OpenCvCameraRotation.UPSIDE_DOWN);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        } else if (RED_AUTO) {
            cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            redWebcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "WebcamLeft"), cameraMonitorViewId);
            redWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    redWebcam.setPipeline(redPipeline);
                    redWebcam.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        }
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
    public YCrCbBlueTeamPropDetectorPipeline.TeamPropPosition getBluePosition() { return bluePipeline.getPosition(); }

    public YCrCbRedTeamPropDetectorPipeline.TeamPropPosition getRedPosition() { return redPipeline.getPosition(); }

    public AprilTagProcessor getAprilTag() { return aprilTag; }
}
