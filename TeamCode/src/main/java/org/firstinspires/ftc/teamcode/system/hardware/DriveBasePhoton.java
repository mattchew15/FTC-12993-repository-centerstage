package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.system.accessory.CoordinatesLogic;
import org.firstinspires.ftc.teamcode.system.accessory.PID;

@Photon
@Config // Allows dashboard to tune
@Deprecated
@SuppressWarnings("unused")
public class DriveBasePhoton
{  // no constructor for this class

    public PhotonDcMotor
            FL,
            FR,
            BL,
            BR;

    public PhotonServo
            ClimbHolderServo,
            DroneServo;

    //variable for the drivebase speed toggle;
    boolean PowerToggled;
    double PowerBase = 1;
    double PowerBaseTurn = 0.88;
    double PowerStrafe = 1.05;

    public static double DroneServoReleasePos = 0.43, DroneServoHoldPos = 0.212;
    public static double ClimbServoReleasePos = 0.4, ClimbServoHoldPos = 0.7;


    public static double DrivebaseXKp = 0.22, DrivebaseXKi = 0.00, DrivebaseXKd = 0.018, DrivebaseXIntegralSumLimit = 10, DrivebaseXKf = 0;
    public static double DrivebaseYKp = 0.22, DrivebaseYKi = 0.00, DrivebaseYKd = 0.018, DrivebaseYIntegralSumLimit = 10, DrivebaseYKf = 0;
    public static double DrivebaseThetaKp = 2, DrivebaseThetaKi = 0.0008, DrivebaseThetaKd = 0.024, DrivebaseThetaIntegralSumLimit = 10, DrivebaseThetaKf = 0;

    // should be able to use one instance of a drivebase pid because the x,y,z translation should all be the same
    PID drivebaseXPID = new PID(DrivebaseXKp,DrivebaseXKi,DrivebaseXKd,DrivebaseXIntegralSumLimit,DrivebaseXKf);
    PID drivebaseYPID = new PID(DrivebaseYKp,DrivebaseYKi,DrivebaseYKd,DrivebaseYIntegralSumLimit,DrivebaseYKf);
    PID drivebaseThetaPID = new PID(DrivebaseThetaKp,DrivebaseThetaKi,DrivebaseThetaKd,DrivebaseThetaIntegralSumLimit,DrivebaseThetaKf);

    CoordinatesLogic coordinatesLogic = new CoordinatesLogic();

    private final double cachingTolerance = 0.005;
    private double previousFrontLeft;
    private double previousBackLeft;
    private double previousFrontRight;
    private double previousBackRight;

    public enum DroneState {
        HOLD,
        RELEASE
    }

    public enum ClimbState {
        HOLD,
        RELEASE
    }

    public void initDrivebase(HardwareMap hwMap){
        FL = hwMap.get(PhotonDcMotor.class, "FL");
        FR = hwMap.get(PhotonDcMotor.class, "FR");
        BL = hwMap.get(PhotonDcMotor.class, "BL");
        BR = hwMap.get(PhotonDcMotor.class, "BR");

        ClimbHolderServo = hwMap.get(PhotonServo.class, "ClimbHolderS");
        DroneServo = hwMap.get(PhotonServo.class, "DroneS");
    }

    // this could be run in robothardware
    public void drivebaseSetup(){
        // zero brake behavior means when motors aren't powered, they will auto brake
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverse correct motors
        //FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE); //DcMotorSimple class?
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        //BR.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setUpFloat()
    {
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void Drive(double LY, double LX, double RX) {
        double denominator = Math.max(Math.abs(LY) + Math.abs(LX) + Math.abs(RX), 1);
        double frontLeftPower = (-LY*PowerBase + LX*PowerStrafe + RX*PowerBaseTurn) / denominator;
        double backLeftPower = (-LY*PowerBase - LX*PowerStrafe + RX*PowerBaseTurn) / denominator;
        double frontRightPower = (-LY*PowerBase - LX*PowerStrafe - RX*PowerBaseTurn) / denominator;
        double backRightPower = (-LY*PowerBase + LX*PowerStrafe - RX*PowerBaseTurn) / denominator;

        // Motor caching runs the motors...
        previousFrontLeft = motorCaching(frontLeftPower, previousFrontLeft, cachingTolerance, FL);

        previousBackLeft = motorCaching(backLeftPower, previousBackLeft, cachingTolerance, BL);

        previousFrontRight = motorCaching(frontRightPower, previousFrontRight, cachingTolerance, FR);

        previousBackRight = motorCaching(backRightPower, previousBackRight, cachingTolerance, BR);
    }



    public void motorDirectionTest(double a, double b, double c, double d){
        FL.setPower(a);
        BL.setPower(b);
        FR.setPower(c);
        BR.setPower(d);
    }
    /*
    public int getMotorPosition(){
        return FL.getCurrentPosition();
    }
     */
    public void runtoPositionTest(int Position){
        FL.setTargetPosition(Position);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(1);
    }

    public void PowerToggle(boolean toggle) { // toggle code for a slow drive mode for fine adjustment
        if (toggle) {
            if (!PowerToggled) {
                if (PowerBase == 1) {

                    PowerStrafe = 0;
                } else {
                    //edit these values to change drivecode

                    PowerStrafe = 1.05;
                }
                PowerToggled = true;
            }
        }
        else {
            PowerToggled = false;
        }
    }

    public void droneState(DroneState state) {
        switch (state) {
            case HOLD:
                DroneServo.setPosition(DroneServoHoldPos);
                break;
            case RELEASE:
                DroneServo.setPosition(DroneServoReleasePos);
                break;
        }
    }
    public void climbState(ClimbState state) {
        switch (state) {
            case HOLD:
                ClimbHolderServo.setPosition(ClimbServoHoldPos);
                break;
            case RELEASE:
                ClimbHolderServo.setPosition(ClimbServoReleasePos);
                break;
        }
    }

    public double motorCaching(double pow, double prev, double cachingTolerance, DcMotor motor)
    {
        if (
                //Should it be >= or > ??
                Math.abs(pow - prev) > cachingTolerance ||
                (pow == 0.0 && prev != 0.0 ) ||
                (pow >= 1.0 && !(prev >= 1.0)) ||
                (pow <= -1.0 && !(prev <= -1.0))
        )
        {
            prev = pow;
            motor.setPower(pow);
        }
        return prev;
    }

}
