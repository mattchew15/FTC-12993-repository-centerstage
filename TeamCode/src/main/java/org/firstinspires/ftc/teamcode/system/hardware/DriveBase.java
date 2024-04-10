package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.EPSILON_DELTA;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.motorCaching;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.system.accessory.CoordinatesLogic;
import org.firstinspires.ftc.teamcode.system.accessory.PID;

@Config // Allows dashboard to tune
public class DriveBase {  // no constructor for this class

    public DcMotorEx
            FL,
            FR,
            BL,
            BR;

    public ServoImplEx DroneServo;

    private double previousFrontLeftPower, previousFrontRightPower, previousBackLeftPower, previousBackRightPower;
    //variable for the drivebase speed toggle;
    boolean PowerToggled;
    double PowerBase = 1;
    double PowerBaseTurn = 0.88;
    double PowerStrafe = 1.05;

    public static double
            DroneServoReleasePos = 0.25,
            DroneServoHoldPos = 0.43;

    public static double DrivebaseXKp = 0.22, DrivebaseXKi = 0.00, DrivebaseXKd = 0.018, DrivebaseXIntegralSumLimit = 10, DrivebaseXKf = 0;
    public static double DrivebaseYKp = 0.22, DrivebaseYKi = 0.00, DrivebaseYKd = 0.018, DrivebaseYIntegralSumLimit = 10, DrivebaseYKf = 0;
    public static double DrivebaseThetaKp = 2, DrivebaseThetaKi = 0.0008, DrivebaseThetaKd = 0.024, DrivebaseThetaIntegralSumLimit = 10, DrivebaseThetaKf = 0;

    // should be able to use one instance of a drivebase pid because the x,y,z translation should all be the same
    PID drivebaseXPID = new PID(DrivebaseXKp,DrivebaseXKi,DrivebaseXKd,DrivebaseXIntegralSumLimit,DrivebaseXKf);
    PID drivebaseYPID = new PID(DrivebaseYKp,DrivebaseYKi,DrivebaseYKd,DrivebaseYIntegralSumLimit,DrivebaseYKf);
    PID drivebaseThetaPID = new PID(DrivebaseThetaKp,DrivebaseThetaKi,DrivebaseThetaKd,DrivebaseThetaIntegralSumLimit,DrivebaseThetaKf);

    CoordinatesLogic coordinatesLogic = new CoordinatesLogic();
    private double powerCoefficient = 2;

    public enum DroneServoState {
        HOLD,
        RELEASE
    }

    public void initDrivebase(HardwareMap hwMap){
        FL = hwMap.get(DcMotorEx.class, "FL");
        FR = hwMap.get(DcMotorEx.class, "FR");
        BL = hwMap.get(DcMotorEx.class, "BL");
        BR = hwMap.get(DcMotorEx.class, "BR");

        DroneServo = hwMap.get(ServoImplEx.class, "DroneS");
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
        //FL.setDirection(DcMotorSimple.Direction.REVERSE); //DcMotorSimple class?
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

        // just comment this out if you don't like the drive...
        /*
        frontLeftPower = Math.pow(frontLeftPower, powerCoefficient);
        backLeftPower = Math.pow(backLeftPower, powerCoefficient);
        frontRightPower = Math.pow(frontRightPower, powerCoefficient);
        backLeftPower = Math.pow(backLeftPower, powerCoefficient);
         */

        previousFrontLeftPower = motorCaching(frontLeftPower, previousFrontLeftPower, EPSILON_DELTA, FL);
        previousFrontRightPower = motorCaching(frontRightPower, previousFrontRightPower, EPSILON_DELTA, FR);
        previousBackLeftPower = motorCaching(backLeftPower, previousBackLeftPower, EPSILON_DELTA, BL);
        previousBackRightPower = motorCaching(backRightPower, previousBackRightPower, EPSILON_DELTA, BR);

    }



    public void motorDirectionTest(double a, double b, double x, double y){
        FL.setPower(a);
        BL.setPower(b);
        FR.setPower(x);
        BR.setPower(y);
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

    public void droneState(DroneServoState state) {
        switch (state) {
            case HOLD:
                DroneServo.setPosition(DroneServoHoldPos);
                break;
            case RELEASE:
                DroneServo.setPosition(DroneServoReleasePos);
                break;
        }
    }
}
