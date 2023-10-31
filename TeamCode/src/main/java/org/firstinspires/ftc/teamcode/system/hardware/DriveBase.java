package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.accessory.PID;

@Config // Allows dashboard to tune
public class DriveBase {  // no constructor for this class

    private DcMotor FR;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FL;
    private DcMotor IntakeMotor;

    //variable for the drivebase speed toggle;
    boolean PowerToggled;
    double PowerBase = 1;
    double PowerBaseTurn = 0.88;
    double PowerStrafe = 1.05;

    public static double DrivebaseXKp = 0.22, DrivebaseXKi = 0.00, DrivebaseXKd = 0.018, DrivebaseXIntegralSumLimit = 10, DrivebaseXKf = 0;
    public static double DrivebaseYKp = 0.22, DrivebaseYKi = 0.00, DrivebaseYKd = 0.018, DrivebaseYIntegralSumLimit = 10, DrivebaseYKf = 0;
    public static double DrivebaseThetaKp = 2, DrivebaseThetaKi = 0.0008, DrivebaseThetaKd = 0.024, DrivebaseThetaIntegralSumLimit = 10, DrivebaseThetaKf = 0;


    // should be able to use one instance of a drivebase pid because the x,y,z translation should all be the same
    PID drivebaseXPID = new PID(DrivebaseXKp,DrivebaseXKi,DrivebaseXKd,DrivebaseXIntegralSumLimit,DrivebaseXKf);
    PID drivebaseYPID = new PID(DrivebaseYKp,DrivebaseYKi,DrivebaseYKd,DrivebaseYIntegralSumLimit,DrivebaseYKf);
    PID drivebaseThetaPID = new PID(DrivebaseThetaKp,DrivebaseThetaKi,DrivebaseThetaKd,DrivebaseThetaIntegralSumLimit,DrivebaseThetaKf);

    CoordinatesLogic coordinatesLogic = new CoordinatesLogic();

    public void Drivebase_init(HardwareMap hwMap) {

        FR = hwMap.get(DcMotor.class, "FR");
        BR = hwMap.get(DcMotor.class, "BR");
        BL = hwMap.get(DcMotor.class, "BL");
        FL = hwMap.get(DcMotor.class, "FL");

        IntakeMotor = hwMap.get(DcMotor.class, "IntakeMotor");

    }

    public void motorsSetup(){
        // zero brake behavior means when motors aren't powered, they will auto brake
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverse correct motors
        //FR.setDirection(DcMotorSimple.Direction.REVERSE); //DcMotorSimple class?
        FL.setDirection(DcMotorSimple.Direction.REVERSE); //DcMotorSimple class?
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        //BR.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); // one intake motor is reversed
    }
    public void motorsSetupAutonomous(){
        // get rid of zero power behavior?? might do something
        //reverse correct motors
        //FR.setDirection(DcMotorSimple.Direction.REVERSE); //DcMotorSimple class?
        FL.setDirection(DcMotorSimple.Direction.REVERSE); //DcMotorSimple class?
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        //BR.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); // one intake motor is reversed
    }


    public void Drive(double LY, double LX, double RX) {
        double denominator = Math.max(Math.abs(LY) + Math.abs(LX) + Math.abs(RX), 1);
        double frontLeftPower = (LY*PowerBase - LX*PowerStrafe + RX*PowerBaseTurn) / denominator;
        double backLeftPower = (LY*PowerBase + LX*PowerStrafe + RX*PowerBaseTurn) / denominator;
        double frontRightPower = (LY*PowerBase + LX*PowerStrafe - RX*PowerBaseTurn) / denominator;
        double backRightPower = (LY*PowerBase - LX*PowerStrafe - RX*PowerBaseTurn) / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        FR.setPower(frontRightPower);
        BR.setPower(backRightPower);
    }

    // this will need to use an inputs class to toggle this state. This will go out of state if the joystick sare moved
    public void DriveToPosition(double xTarget, double yTarget, double thetaTarget, double xRobotPosition, double yRobotPosition, double robotTheta, double maxTranslationalSpeed, double maxRotationalSpeed){
        double x = drivebaseXPID.update(xTarget,xRobotPosition,maxTranslationalSpeed); // set a target, get the robots state, and set the max speed
        double y = drivebaseYPID.update(yTarget,-yRobotPosition,maxTranslationalSpeed);
        double theta = drivebaseThetaPID.update(thetaTarget,-robotTheta,maxRotationalSpeed); // this pid should probably be different
        double x_rotated = x * Math.cos(robotTheta) - y * Math.sin(robotTheta);
        double y_rotated = x * Math.sin(robotTheta) + y * Math.cos(robotTheta);

        // x, y, theta input mixing
        double denominator = Math.max(Math.abs(x_rotated) + Math.abs(y_rotated) + Math.abs(robotTheta), 1);
        FL.setPower((x_rotated + y_rotated + theta)/denominator);
        BL.setPower((x_rotated - y_rotated + theta)/denominator);
        FR.setPower((x_rotated - y_rotated - theta)/denominator);
        BR.setPower((x_rotated + y_rotated - theta)/denominator);
    }
    public void DriveToPositionAutonomous(double xTarget, double yTarget, double thetaTarget, double xRobotPosition, double yRobotPosition, double robotTheta, double maxTranslationalSpeed, double maxRotationalSpeed){
        double x = drivebaseXPID.update(xTarget,xRobotPosition,maxTranslationalSpeed); // set a target, get the robots state, and set the max speed
        double y = -drivebaseYPID.update(yTarget,yRobotPosition,maxTranslationalSpeed);
        double theta = -drivebaseThetaPID.update(thetaTarget,robotTheta,maxRotationalSpeed); // this pid should probably be different
        double x_rotated = x * Math.cos(robotTheta) - y * Math.sin(robotTheta);
        double y_rotated = x * Math.sin(robotTheta) + y * Math.cos(robotTheta);

        // x, y, theta input mixing
        double denominator = Math.max(Math.abs(x_rotated) + Math.abs(y_rotated) + Math.abs(robotTheta), 1);
        FL.setPower((x_rotated + y_rotated + theta));
        BL.setPower((x_rotated - y_rotated + theta));
        FR.setPower((x_rotated - y_rotated - theta));
        BR.setPower((x_rotated + y_rotated - theta));
    }
    public void DriveToPositionAutonomous2(double xTarget, double yTarget, double thetaTarget, double xRobotPosition, double yRobotPosition, double robotThetaOffset,double robotTheta, double maxTranslationalSpeed, double maxRotationalSpeed){
        double x = drivebaseXPID.update(xTarget,xRobotPosition,maxTranslationalSpeed); // set a target, get the robots state, and set the max speed
        double y = -drivebaseYPID.update(yTarget,yRobotPosition,maxTranslationalSpeed);
        double theta = drivebaseThetaPID.update(thetaTarget,robotThetaOffset,maxRotationalSpeed); // this pid should probably be different
        double x_rotated = x * Math.cos(robotTheta) - y * Math.sin(robotTheta);
        double y_rotated = x * Math.sin(robotTheta) + y * Math.cos(robotTheta);

        // x, y, theta input mixing
        double denominator = Math.max(Math.abs(x_rotated) + Math.abs(y_rotated) + Math.abs(robotTheta), 1);
        FL.setPower((x_rotated + y_rotated + theta));
        BL.setPower((x_rotated - y_rotated + theta));
        FR.setPower((x_rotated - y_rotated - theta));
        BR.setPower((x_rotated + y_rotated - theta));
    } // change from doubles to poses if wanting to optimize
    public void DriveToPositionAutonomous3(double stackTargetX, double stackTargetY, double xTarget, double yTarget, double xRobotPosition, double yRobotPosition,double robotTheta, double maxTranslationalSpeed, double maxRotationalSpeed, Telemetry telemetry, Pose2d currentPose){
        double x = drivebaseXPID.update(xTarget,xRobotPosition,maxTranslationalSpeed); // set a target, get the robots state, and set the max speed
        double y = -drivebaseYPID.update(yTarget,yRobotPosition,maxTranslationalSpeed);
        double thetaPower = coordinatesLogic.pointToTarget(new Pose2d(stackTargetX,stackTargetY,0),currentPose);
        telemetry.addData("thetapower", thetaPower);
        double theta = -drivebaseThetaPID.updateWithError(thetaPower, maxRotationalSpeed); // this pid should probably be different
        telemetry.addData("theta", theta);
        double x_rotated = x * Math.cos(robotTheta) - y * Math.sin(robotTheta);
        double y_rotated = x * Math.sin(robotTheta) + y * Math.cos(robotTheta);

        // x, y, theta input mixing
        // double denominator = Math.max(Math.abs(x_rotated) + Math.abs(y_rotated) + Math.abs(robotTheta), 1);

        FL.setPower((x_rotated + y_rotated + theta));
        BL.setPower((x_rotated - y_rotated + theta));
        FR.setPower((x_rotated - y_rotated - theta));
        BR.setPower((x_rotated + y_rotated - theta));
    }

    public double holdHeading(double thetaTarget, double robotTheta, double maxRotationalSpeed){
        double thetaOutput = -drivebaseThetaPID.update(thetaTarget,robotTheta,maxRotationalSpeed); // this pid should probably be different
        return thetaOutput;
    }

    public double getDistanceFromPosition(double xTarget, double yTarget, double thetaTarget, double xRobotPosition, double yRobotPosition, double robotTheta){
        double xDistance = xTarget - xRobotPosition;
        double yDistance = yTarget - yRobotPosition;
        double distance = Math.sqrt(Math.pow(xDistance,2) + Math.pow(yDistance,2));
        return distance;
    }

    public double getXError(){
        return drivebaseXPID.returnError();
    }
    public double getYError(){
        return drivebaseYPID.returnError();
    }
    public double getHeadingError(){
        return drivebaseThetaPID.returnError();
    }
    //  public double getHeadingToPointError(){
    //    return drivebaseThetaPID.returnError();
    // }

    public double getXOutput(){
        return drivebaseXPID.returnOutput();
    }
    public double getYOutput(){
        return drivebaseYPID.returnOutput();
    }
    public double getHeadingOutput(){
        return drivebaseThetaPID.returnOutput();
    }

    public void ResetDriveToPosition(){ // this should probably be in stormdrive

    }

    public void motorDirectionTest(double a, double b, double c, double d){
        FL.setPower(a);
        BL.setPower(b);
        FR.setPower(c);
        BR.setPower(d);
    }
    public int getMotorPosition(){
        return FL.getCurrentPosition();
    }
    public void runtoPositionTest(int Position){
        FL.setTargetPosition(Position);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setPower(1);
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

    public void intakeSpin(double speedDirection){
        IntakeMotor.setPower(speedDirection);
    }

}
