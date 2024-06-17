package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveThread;
import org.firstinspires.ftc.teamcode.system.hardware.CameraHardware;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.teamPropLocation;
import static org.firstinspires.ftc.teamcode.system.hardware.SetAuto.setBlueAuto;
@Disabled
@TeleOp(name = "Sensors HealthCheck")
public class FullSystemTest extends LinearOpMode
{

    // this was like never use lol
    enum TestCase
    {
        IDLE,
        LEFT_LIMIT_SWITCH,
        RIGHT_LIMIT_SWITCH,
        BACK_COLOUR_SENSOR,
        FRONT_COLOUR_SENSOR,
        DISTANCE_SENSOR,
        SIDE_CAMERA,
        BACK_CAMERA,
        END
    }
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    DriveBase driveBase = new DriveBase(telemetry);
    TestCase testCase = TestCase.IDLE;
    CameraHardware cameraHardware = new CameraHardware();
    boolean isDriveWorking, isDistanceSensorWorking, isBackColourSensorWorking,
            isFrontColourSensorWorking, isLeftLimitSwitchWorking, isRightLimitSwitchWorking,
            isSideCameraWorking, isBackCameraWorking;
    boolean front, middle, back;
    @Override
    public void runOpMode() throws InterruptedException
    {
        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();
        outtakeSubsystem.hardwareSetup();
        intakeSubsystem.intakeHardwareSetup();
        outtakeSubsystem.cacheInitialPitchValue();
        SampleMecanumDriveThread sampleMecanumDrive = new SampleMecanumDriveThread(hardwareMap);
        sampleMecanumDrive.startImuThread(this);

        setBlueAuto();

        waitForStart();

        while (opModeIsActive())
        {

            switch (testCase)
            {

                case IDLE:
                    telemetry.addLine("Press A to start");
                    if (gamepad1.a)
                    {
                        testCase = TestCase.LEFT_LIMIT_SWITCH;
                    }
                    break;
                case LEFT_LIMIT_SWITCH:
                    telemetry.addLine("Click the left limit switch, or press X to skip");
                    telemetry.addData("Left limit switch", !intakeSubsystem.leftArmLimitSwitchValue);
                    if (!intakeSubsystem.leftArmLimitSwitchValue)
                    {
                        testCase = TestCase.RIGHT_LIMIT_SWITCH;
                        isLeftLimitSwitchWorking = true;
                    }
                    if (gamepad1.x)
                    {
                        testCase = TestCase.RIGHT_LIMIT_SWITCH;
                        isLeftLimitSwitchWorking = false;
                    }
                    break;
                case RIGHT_LIMIT_SWITCH:
                    telemetry.addLine("Click the right limit switch, or press B to skip");
                    telemetry.addData("Right limit switch", !intakeSubsystem.leftArmLimitSwitchValue);
                    if (!intakeSubsystem.rightArmLimitSwitchValue)
                    {
                        testCase = TestCase.BACK_COLOUR_SENSOR;
                        isRightLimitSwitchWorking = true;
                    }
                    if (gamepad1.b)
                    {
                        testCase = TestCase.BACK_COLOUR_SENSOR;
                        isRightLimitSwitchWorking = false;
                    }
                    break;
                case BACK_COLOUR_SENSOR:
                    telemetry.addLine("Place a pixel in the back colour sensor, or press X to skip");
                    telemetry.addData("BackColourSensor", intakeSubsystem.backColourSensorValue);
                    if (intakeSubsystem.backColourSensorValue > 1000)
                    {
                        testCase = TestCase.BACK_COLOUR_SENSOR;
                        isBackColourSensorWorking = true;
                    }
                    if (gamepad1.x)
                    {
                        testCase = TestCase.BACK_COLOUR_SENSOR;
                        isBackColourSensorWorking = false;
                    }
                    break;
                case FRONT_COLOUR_SENSOR:
                    telemetry.addLine("Place a pixel in the front colour sensor, or press B to skip");
                    telemetry.addData("Front colour sensor", intakeSubsystem.frontColourSensorValue);
                    if (intakeSubsystem.frontColourSensorValue > 1000)
                    {
                        testCase = TestCase.DISTANCE_SENSOR;
                        isFrontColourSensorWorking = true;
                    }
                    if (gamepad1.b)
                    {
                        testCase = TestCase.DISTANCE_SENSOR;
                        isFrontColourSensorWorking = false;
                    }
                    break;
                case DISTANCE_SENSOR:
                    telemetry.addLine("Place something in the distance sensor, or press X to skip");
                    telemetry.addData("Distance sensor", intakeSubsystem.frontColourSensorValue);
                    if (intakeSubsystem.frontColourSensorValue > 1000)
                    {
                        testCase = TestCase.SIDE_CAMERA;
                        isDistanceSensorWorking = true;
                        cameraHardware.initWebcam(hardwareMap, telemetry);
                        teamPropLocation = 0;
                    }
                    if (gamepad1.b)
                    {
                        testCase = TestCase.SIDE_CAMERA;
                        isDistanceSensorWorking = false;
                        cameraHardware.initWebcam(hardwareMap, telemetry);
                        teamPropLocation = 0;
                    }
                    break;
                case SIDE_CAMERA:
                    telemetry.addLine("Test all preload positions, or press x to skip");
                    telemetry.addData("Front", front);
                    telemetry.addData("Middle", middle);
                    telemetry.addData("Back", back);
                    if (teamPropLocation == 1)
                    {
                        front = true;
                    }
                    if (teamPropLocation == 2)
                    {
                        middle = true;
                    }
                    if (teamPropLocation == 3)
                    {
                        back = true;
                    }
                    if (front && middle && back)
                    {
                        testCase = TestCase.BACK_CAMERA;
                        isSideCameraWorking = true;
                        cameraHardware.closeWebcam();
                        cameraHardware.initBackWebcamVP(hardwareMap, telemetry);
                    }
                    if (gamepad1.x)
                    {
                        testCase = TestCase.BACK_CAMERA;
                        isSideCameraWorking = false;
                        cameraHardware.closeWebcam();
                        cameraHardware.initBackWebcamVP(hardwareMap, telemetry);
                    }
                    break;
                case BACK_CAMERA:
                    double num = cameraHardware.getNumSeenTags();
                    telemetry.addLine("Test back camera, or press b to skip");
                    telemetry.addData("Num AT being seen", num);
                    if (num > 0)
                    {
                        testCase = TestCase.END;
                        isBackCameraWorking = true;
                        cameraHardware.closeBackWebcam();
                    }
                    if (gamepad1.b)
                    {
                        testCase = TestCase.END;
                        isBackCameraWorking = false;
                        cameraHardware.closeBackWebcam();
                    }
                    break;

                case END:
                    telemetry.addLine("End of process");
                    if (isDistanceSensorWorking && isBackColourSensorWorking && isFrontColourSensorWorking && isLeftLimitSwitchWorking && isRightLimitSwitchWorking && isSideCameraWorking && isBackCameraWorking)
                    {
                        telemetry.addLine("Everything works yeahhh!!! just win worlds now");
                    } else
                    {
                        telemetry.addLine("Something doesn't work :(, still win worlds?");
                        telemetry.addLine("");
                        telemetry.addData("isDistanceSensorWorking", isDistanceSensorWorking);
                        telemetry.addData("isBackColourSensorWorking", isBackColourSensorWorking);
                        telemetry.addData("isFrontColourSensorWorking", isFrontColourSensorWorking);
                        telemetry.addData("isLeftLimitSwitchWorking", isLeftLimitSwitchWorking);
                        telemetry.addData("isRightLimitSwitchWorking", isRightLimitSwitchWorking);
                        telemetry.addData("isSideCameraWorking", isSideCameraWorking);
                        telemetry.addData("isBackCameraWorking", isBackCameraWorking);
                    }
                    break;
            }
            telemetry.addData("Case", testCase);
            telemetry.update();


        }

    }
}
