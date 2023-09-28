package org.firstinspires.ftc.teamcode.system.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.system.vision.BlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.vision.RedTeamPropDetectorPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
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

    public Servo
            bottomRoller,
            brushHeight,
            flap,
            rail,
            armLeft,
            armRight,
            pivot,
            wrist,
            claw,
            outtakeLock,
            intakeLock;

    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    private OpenCvWebcam blueWebcam;
    private OpenCvWebcam redWebcam;

    public BlueTeamPropDetectorPipeline bluePipeline = new BlueTeamPropDetectorPipeline();
    public RedTeamPropDetectorPipeline redPipeline;
    int cameraMonitorViewId;

    public void init(HardwareMap hwMap) {
        frontLeftDrive = hwMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightDrive = hwMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftDrive = hwMap.get(DcMotorEx.class, "backLeftMotor");
        backRightDrive = hwMap.get(DcMotorEx.class, "backRightMotor");
        extension = hwMap.get(DcMotorEx.class, "extensionMotor");
        intake = hwMap.get(DcMotorEx.class, "intake");
        lift = hwMap.get(DcMotorEx.class, "lift");
        pitch = hwMap.get(DcMotorEx.class, "pitch");

        bottomRoller = hwMap.get(Servo.class, "Bottom_Roller");
        brushHeight = hwMap.get(Servo.class, "Brush_Height");
        flap = hwMap.get(Servo.class, "Flap");
        rail = hwMap.get(Servo.class, "Rail");
        armLeft = hwMap.get(Servo.class, "Arm_Left");
        armRight = hwMap.get(Servo.class, "Arm_Right");
        pivot = hwMap.get(Servo.class, "Pivot");
        wrist = hwMap.get(Servo.class, "Wrist");
        claw = hwMap.get(Servo.class, "Claw");
        outtakeLock = hwMap.get(Servo.class, "Outtake_Lock");
        intakeLock = hwMap.get(Servo.class, "Intake_Lock");
    }
}
