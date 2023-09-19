package org.firstinspires.ftc.teamcode.system.hardware;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.system.vision.BlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.RedTeamPropDetectorPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class VisionHardware {

    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    public OpenCvWebcam blueWebcam;
    public OpenCvWebcam redWebcam;

    public BlueTeamPropDetectorPipeline blueTeamPropPipeline;
    public RedTeamPropDetectorPipeline redTeamPropPipeline;
    int cameraMonitorViewId;

    public void init(HardwareMap hwMap) {
        if (Globals.auto) { //If auto init backWebcam
            aprilTag = new AprilTagProcessor.Builder()
                    .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .addProcessor(aprilTag)
                    .setCamera(hwMap.get(WebcamName.class, "backWebcam"))
                    .setCameraResolution(new Size(640, 480))
                    .enableLiveView(true)
                    .build();

            cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier
                    ("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());

            if (Globals.blueAuto) { // If blue auto, init blueWebcam
                blueTeamPropPipeline = new BlueTeamPropDetectorPipeline();
                blueWebcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "blueWebcam"), cameraMonitorViewId);
                blueWebcam.setPipeline(blueTeamPropPipeline);
                blueWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        blueWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {
                    }
                });
            } else if (Globals.redAuto) { // If red auto, init redWebcam
                redTeamPropPipeline = new RedTeamPropDetectorPipeline();
                redWebcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "redWebcam"), cameraMonitorViewId);
                redWebcam.setPipeline(redTeamPropPipeline);
                redWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        redWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {
                    }
                });
            }

        }
    }
}
