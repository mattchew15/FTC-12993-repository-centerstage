package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.system.vision.BlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.RedTeamPropDetectorPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class RobotHardware {
    public DcMotorEx
            frontLeftDrive,
            frontRightDrive,
            backLeftDrive,
            backRightDrive,
            extension,
            intake,
            lift,
            pitch;

    public ServoImplEx
            bottomRoller,
            brushHeight,
            flap,
            intakeLock,
            rail,
            armLeft,
            armRight,
            pivot,
            wrist,
            claw,
            outtakeLock,
            drone;

    int cameraMonitorViewId;
    private OpenCvWebcam blueWebcam, redWebcam;
    public BlueTeamPropDetectorPipeline bluePipeline = new BlueTeamPropDetectorPipeline();
    public RedTeamPropDetectorPipeline redPipeline = new RedTeamPropDetectorPipeline();
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;

    public void init(HardwareMap hwMap) {
        frontLeftDrive = hwMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightDrive = hwMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftDrive = hwMap.get(DcMotorEx.class, "backLeftMotor");
        backRightDrive = hwMap.get(DcMotorEx.class, "backRightMotor");
        extension = hwMap.get(DcMotorEx.class, "extensionMotor");
        intake = hwMap.get(DcMotorEx.class, "intake");
        lift = hwMap.get(DcMotorEx.class, "lift");
        pitch = hwMap.get(DcMotorEx.class, "pitch");



        bottomRoller = hwMap.get(ServoImplEx.class, "Bottom_Roller");
        brushHeight = hwMap.get(ServoImplEx.class, "Brush_Height");
        flap = hwMap.get(ServoImplEx.class, "Flap");
        intakeLock = hwMap.get(ServoImplEx.class, "Intake_Lock");
        rail = hwMap.get(ServoImplEx.class, "Rail");
        armLeft = hwMap.get(ServoImplEx.class, "Arm_Left");
        armRight = hwMap.get(ServoImplEx.class, "Arm_Right");
        pivot = hwMap.get(ServoImplEx.class, "Pivot");
        wrist = hwMap.get(ServoImplEx.class, "Wrist");
        claw = hwMap.get(ServoImplEx.class, "Claw");
        outtakeLock = hwMap.get(ServoImplEx.class, "Outtake_Lock");
        drone = hwMap.get(ServoImplEx.class, "Drone");

        initWebcam(hwMap);
    }

    public void initWebcam(HardwareMap hwMap) {
        if (BLUE_AUTO) {
            cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            blueWebcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            blueWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    blueWebcam.setPipeline(bluePipeline);
                    blueWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        } else if (RED_AUTO) {
            cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            redWebcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            redWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    redWebcam.setPipeline(redPipeline);
                    redWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        }
    }

    public void initAprilTag(HardwareMap hwMap) {
        if (BLUE_AUTO) {
            blueWebcam.closeCameraDevice();
        } else if (RED_AUTO) {
            redWebcam.closeCameraDevice();
        }

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hwMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(false)
                .build();
    }

    public BlueTeamPropDetectorPipeline.TeamPropPosition getBluePosition() { return bluePipeline.getPosition(); }

    public RedTeamPropDetectorPipeline.TeamPropPosition getRedPosition() { return redPipeline.getPosition(); }

    public AprilTagProcessor getAprilTag() { return aprilTag; }

}
