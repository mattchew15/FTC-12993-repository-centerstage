package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.EPSILON_DELTA;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.motorCaching;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    // higher values of k means more adjustment.
    // c centers the adjustment

//    k = 20, c = 0.7
//    x = 0.1, y = 0.000017
//    x = 0.3, y = 0.000911
//    x = 0.5, y = 0.047426
//    x = 0.7, y = 0.500000
//    x = 0.9, y = 0.977023

//    k = 5, c = 0.3
//    x = 0.1, y = 0.377541
//    x = 0.3, y = 0.500000
//    x = 0.5, y = 0.622459
//    x = 0.7, y = 0.750000
//    x = 0.9, y = 0.845534

    // where the scaling centers around
    public static double c = 1.7;
    // how agressive the scaling is
    public static double m = 1;

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
    Telemetry telemetry;
    private double powerCoefficient = 2;

    public enum DroneServoState {
        HOLD,
        RELEASE
    }

    public DriveBase(Telemetry t){
        this.telemetry = t;
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

    public static double adjustedJoystick(double x) {
        double y = Math.pow(c-x,m);
       // double y = Math.pow(x,c-x);
        return Math.pow(x,y);
    }

    public void Drive(double LY, double LX, double RX) {
//        double f = LY < 0? -adjustedJoystick(Math.abs(LY)):adjustedJoystick(Math.abs(LY));
//        double s = LX < 0? -adjustedJoystick(Math.abs(LX)):adjustedJoystick(Math.abs(LX));
//        double t = RX < 0? -adjustedJoystick(Math.abs(RX)):adjustedJoystick(Math.abs(RX));

        double s = LX;
        double t = RX;
        double f = LY;

        telemetry.addData("LY",LY);
        telemetry.addData("f",f);
        telemetry.addData("LX",LX);
        telemetry.addData("s",s);
        telemetry.addData("RX",RX);
        telemetry.addData("t",t);

        double denominator = Math.max(Math.abs(LY) + Math.abs(LX) + Math.abs(RX), 1);
        double frontLeftPower = (-f*PowerBase + s*PowerStrafe + t*PowerBaseTurn) / denominator;
        double backLeftPower = (-f*PowerBase - s*PowerStrafe + t*PowerBaseTurn) / denominator;
        double frontRightPower = (-f*PowerBase - s*PowerStrafe - t*PowerBaseTurn) / denominator;
        double backRightPower = (-f*PowerBase + s*PowerStrafe - t*PowerBaseTurn) / denominator;

        // just comment this out if you don't like the drive...

//        frontLeftPower = Math.pow(frontLeftPower, powerCoefficient);
//        backLeftPower = Math.pow(backLeftPower, powerCoefficient);
//        frontRightPower = Math.pow(frontRightPower, powerCoefficient);
//        backLeftPower = Math.pow(backLeftPower, powerCoefficient);


        previousFrontLeftPower = motorCaching(frontLeftPower, previousFrontLeftPower, EPSILON_DELTA, FL);
        previousFrontRightPower = motorCaching(frontRightPower, previousFrontRightPower, EPSILON_DELTA, FR);
        previousBackLeftPower = motorCaching(backLeftPower, previousBackLeftPower, EPSILON_DELTA, BL);
        previousBackRightPower = motorCaching(backRightPower, previousBackRightPower, EPSILON_DELTA, BR);

    }



    public void motorDirectionTest(double FL, double FR, double BL, double BR){
        this.FL.setPower(FL);
        this.BL.setPower(BL);
        this.FR.setPower(FR);
        this.BR.setPower(BR);
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
