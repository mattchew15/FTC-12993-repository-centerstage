package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.accessory.CoordinatesLogic;
import org.firstinspires.ftc.teamcode.system.accessory.PID;

@Config // Allows dashboard to tune
public class DriveBase {  // no constructor for this class

    RobotHardware robotHardware;

    //variable for the drivebase speed toggle;
    boolean PowerToggled;
    double PowerBase = 1;
    double PowerBaseTurn = 0.88;
    double PowerStrafe = 1.05;

    public static double DroneServoReleasePos = 0.43, DroneServoHoldPos = 0.212;

    public static double DrivebaseXKp = 0.22, DrivebaseXKi = 0.00, DrivebaseXKd = 0.018, DrivebaseXIntegralSumLimit = 10, DrivebaseXKf = 0;
    public static double DrivebaseYKp = 0.22, DrivebaseYKi = 0.00, DrivebaseYKd = 0.018, DrivebaseYIntegralSumLimit = 10, DrivebaseYKf = 0;
    public static double DrivebaseThetaKp = 2, DrivebaseThetaKi = 0.0008, DrivebaseThetaKd = 0.024, DrivebaseThetaIntegralSumLimit = 10, DrivebaseThetaKf = 0;

    // should be able to use one instance of a drivebase pid because the x,y,z translation should all be the same
    PID drivebaseXPID = new PID(DrivebaseXKp,DrivebaseXKi,DrivebaseXKd,DrivebaseXIntegralSumLimit,DrivebaseXKf);
    PID drivebaseYPID = new PID(DrivebaseYKp,DrivebaseYKi,DrivebaseYKd,DrivebaseYIntegralSumLimit,DrivebaseYKf);
    PID drivebaseThetaPID = new PID(DrivebaseThetaKp,DrivebaseThetaKi,DrivebaseThetaKd,DrivebaseThetaIntegralSumLimit,DrivebaseThetaKf);

    CoordinatesLogic coordinatesLogic = new CoordinatesLogic();

    public enum DroneState {
        HOLD,
        RELEASE
    }

    // this could be run in robothardware
    public void motorsSetup(){
        // zero brake behavior means when motors aren't powered, they will auto brake
        robotHardware.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverse correct motors
        //FR.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.FL.setDirection(DcMotorSimple.Direction.REVERSE); //DcMotorSimple class?
        robotHardware.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        //BR.setDirection(DcMotorSimple.Direction.REVERSE);
        robotHardware.IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); // reveresed apparently
    }

    public void Drive(double LY, double LX, double RX) {
        double denominator = Math.max(Math.abs(LY) + Math.abs(LX) + Math.abs(RX), 1);
        double frontLeftPower = (LY*PowerBase - LX*PowerStrafe + RX*PowerBaseTurn) / denominator;
        double backLeftPower = (LY*PowerBase + LX*PowerStrafe + RX*PowerBaseTurn) / denominator;
        double frontRightPower = (LY*PowerBase + LX*PowerStrafe - RX*PowerBaseTurn) / denominator;
        double backRightPower = (LY*PowerBase - LX*PowerStrafe - RX*PowerBaseTurn) / denominator;

        robotHardware.FL.setPower(frontLeftPower);
        robotHardware.BL.setPower(backLeftPower);
        robotHardware.FR.setPower(frontRightPower);
        robotHardware.BR.setPower(backRightPower);
    }



    public void motorDirectionTest(double a, double b, double c, double d){
        robotHardware.FL.setPower(a);
        robotHardware.BL.setPower(b);
        robotHardware.FR.setPower(c);
        robotHardware.BR.setPower(d);
    }
    /*
    public int getMotorPosition(){
        return robotHardware.FL.getCurrentPosition();
    }
     */
    public void runtoPositionTest(int Position){
        robotHardware.FL.setTargetPosition(Position);
        robotHardware.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardware.FL.setPower(1);
    }

    public void PowerToggle(boolean toggle) { // toggle code for a slow drive mode for fine adjustment
        if (toggle) {
            if (!PowerToggled) {
                if (PowerBase == 1) {
                    PowerBase = 0.33;
                    PowerBaseTurn = 0.3;
                    PowerStrafe = 0.36;
                } else {
                    //edit these values to change drivecode
                    PowerBase = 1;
                    PowerBaseTurn = 0.8;
                    PowerStrafe = 1.05;
                }
                PowerToggled = true;
            }
        }
        else {
            PowerToggled = false;
        }
    }
    /*
    public void droneState(DroneState state) {
        switch (state) {
            case HOLD:
                robotHardware.DroneSero.setPosition(DroneServoHoldPos);
                break;
            case RELEASE:
                robotHardware.DroneServo.setPosition(DroneServoReleasePos);
                break;
        }
    }
     */
}
