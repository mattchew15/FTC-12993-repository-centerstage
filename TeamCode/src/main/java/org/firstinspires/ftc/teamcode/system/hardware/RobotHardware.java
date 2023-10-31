package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import android.util.Size;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
            FL,
            FR,
            BL,
            BR,
            IntakeSlideMotor,
            IntakeMotor,
            LiftMotor,
            PitchMotor;

    public CRServo BottomRollerServo;

    public ServoImplEx
            IntakeArmServo,
            IntakeFlapServo,
            IntakeClip,
            OuttakeArmServoLeft,
            OuttakeArmServoRight,
            PivotServo,
            WristServo,
            ClawServo,
            DroneServo;

    private OpenCvWebcam blueWebcam, redWebcam;
    private final BlueTeamPropDetectorPipeline bluePipeline = new BlueTeamPropDetectorPipeline();
    private final RedTeamPropDetectorPipeline redPipeline = new RedTeamPropDetectorPipeline();
    private AprilTagProcessor aprilTag;
    
    public void initializeHardware(HardwareMap hwMap){
        initMotorsServos(hwMap);
        initWebcam(hwMap);
        initAprilTag(hwMap);
    }

    public void initMotorsServos(HardwareMap hwMap) {
        FL = hwMap.get(DcMotorEx.class, "FL");
        FR = hwMap.get(DcMotorEx.class, "FR");
        BL = hwMap.get(DcMotorEx.class, "BL");
        BR = hwMap.get(DcMotorEx.class, "BR");
        IntakeSlideMotor = hwMap.get(DcMotorEx.class, "IntakeSLideMotor");
        IntakeMotor = hwMap.get(DcMotorEx.class, "IntakeMotor");
        LiftMotor = hwMap.get(DcMotorEx.class, "LiftMotor");
        PitchMotor = hwMap.get(DcMotorEx.class, "PitchMotor");

        BottomRollerServo = hwMap.get(CRServo.class, "BottomRollerS");

        IntakeArmServo = hwMap.get(ServoImplEx.class, "IntakeArmS");
        IntakeFlapServo = hwMap.get(ServoImplEx.class, "IntakeFlapS");
        IntakeClip = hwMap.get(ServoImplEx.class, "IntakeClip");
        OuttakeArmServoLeft = hwMap.get(ServoImplEx.class, "ArmSLeft");
        OuttakeArmServoRight = hwMap.get(ServoImplEx.class, "ArmSRight");
        PivotServo = hwMap.get(ServoImplEx.class, "PivotS");
        WristServo = hwMap.get(ServoImplEx.class, "WristS");
        ClawServo = hwMap.get(ServoImplEx.class, "ClawS");
        DroneServo = hwMap.get(ServoImplEx.class, "DroneS");

        //OuttakeArmServoLeft.setDirection(Servo.Direction.REVERSE);
    }
    
    public void initWebcam(HardwareMap hwMap) {
        
        int cameraMonitorViewId;
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
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hwMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(false)
                .build();
    }

    
    //Pipeline has to be in the same class as where webcam stuff is initialized
    public BlueTeamPropDetectorPipeline.TeamPropPosition getBluePosition() { return bluePipeline.getPosition(); }
    public double getBlueCx() { return bluePipeline.getCx();}
    public double getBlueCy() { return bluePipeline.getCy();}

    public RedTeamPropDetectorPipeline.TeamPropPosition getRedPosition() { return redPipeline.getPosition(); }
    public double getRedCx() { return redPipeline.getCx();}
    public double getRedCy() { return redPipeline.getCy();}

    public AprilTagProcessor getAprilTag() { return aprilTag; }
}
