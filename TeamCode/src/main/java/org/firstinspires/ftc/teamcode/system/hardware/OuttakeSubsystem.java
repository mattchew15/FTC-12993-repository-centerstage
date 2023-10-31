package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.system.accessory.PID;

@Config
public class OuttakeSubsystem {

    RobotHardware robotHardware;

    public static double ArmLeftReadyPos = 0.43, ArmLeftTransferPos = 0.212, ArmLeftScorePos = 0, ArmLeftScoreUpPos = 0;
    public static double ArmRightReadyPos = 0.43, ArmRightTransferPos = 0.212, ArmRightScorePos = 0, ArmRightScoreUpPos = 0;

    public static double PivotReadyPos = 0.145, PivotDiagonalLeftPos = 0.241, PivotDiagonalRightPos = 0.32, PivotDiagonalLeftFlippedPos = 0, PivotDiagonalRightFlippedPos;
    public static double PivotSidewaysLeftPos = 0.145, PivotSidewaysRightPos = 0;
    public static double WristReadyPos = 0.43, WristTransferPos = 0.212, WristScorePos;
    public static double ClawOpenPos = 0.7, ClawClosedPos = 0.3;

    public static double LiftKp = 0.015, LiftKi = 0.0001, LiftKd = 0.00006, LiftIntegralSumLimit = 10, LiftKf = 0;
    public static double PitchKp = 0.007, PitchKi = 0.000, PitchKd = 0.0002, PitchIntegralSumLimit = 1, PitchFeedforward = 0.3;

    PID liftPID = new PID(LiftKp,LiftKi,LiftKd,LiftIntegralSumLimit,LiftKf);
    PID pitchPID = new PID(PitchKp,PitchKi,PitchKd,PitchIntegralSumLimit,PitchFeedforward);

    final double pitchthresholdDistance = degreestoTicksPitchMotor(2);
    final double liftthresholdDistance = 22;

    double pitchTarget;
    int liftTarget;

    public double pitchPosition;
    public double liftPosition;


    //Servo stuff
    public enum ArmServoState {
        READY,
        TRANSFER,
        SCORE,
        SCORE_UP
    }

    public enum PivotState {
        READY,
        DIAGONAL_LEFT,
        DIAGONAL_RIGHT,
        DIAGONAL_LEFT_FLIPPED,
        DIAGONAL_RIGHT_FLIPPED,
        SIDEWAYS_LEFT,
        SIDEWAYS_RIGHT
    }

    public enum WristState {
        READY,
        TRANSFER,
        SCORE
    }

    public enum ClawState { // might add more states here
        CLOSE,
        OPEN
    }

    public void hardwareSetup(){
        robotHardware.PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardware.LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // run without encoder is if using external PID

        //OuttakeArmServo.setDirection(Servo.Direction.REVERSE);
        //turretPosition = 0; // these need to be initialized on setup or you will get null error??
        //liftPosition = 0;
        //intakeSlidePosition = 0;
    }

    public void outtakeReads(){
        pitchPosition = robotHardware.PitchMotor.getCurrentPosition();
        liftPosition = robotHardware.LiftMotor.getCurrentPosition();

        //other things like distance sensors etc
    }

    public void encodersReset(){
        robotHardware.PitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void liftMotorRawControl(double manualcontrollift){
        robotHardware.LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this is a write that is not needed
        robotHardware.LiftMotor.setPower(manualcontrollift * 0.75);
    }

    public void pitchMotorRawControl(double manualcontrolturret){
        robotHardware.PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardware.PitchMotor.setPower(manualcontrolturret * -0.4);
    }

    public void liftTo(int rotations, double motorPosition, double maxSpeed){
        liftTarget = rotations;
        robotHardware.LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this is added so that the external pids could be used
        double output = liftPID.update(liftTarget,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        robotHardware.LiftMotor.setPower(output);
    }

    public void pitchTo(double targetRotations, double motorPosition, double maxSpeed){
        pitchTarget = targetRotations;
        robotHardware.PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double output = pitchPID.update(targetRotations,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        robotHardware.PitchMotor.setPower(output);
    }

    public boolean liftTargetReached(){
        if (liftPosition > (liftTarget - liftthresholdDistance) && liftPosition < (liftTarget+liftthresholdDistance)){ //liftthresholdDistance
            return true;
        }
        else{
            return false;
        }
    }

    public boolean pitchTargetReached(){
        if (pitchPosition < (pitchTarget + pitchthresholdDistance) && pitchPosition > (pitchTarget-pitchthresholdDistance)){
            return true;
        }
        else{
            return false;
        }
    }

    public void ArmServoState(ArmServoState state) {
        switch (state) {
            case READY:
                robotHardware.OuttakeArmServoRight.setPosition(ArmRightReadyPos);
                robotHardware.OuttakeArmServoLeft.setPosition(ArmLeftReadyPos);
            break;
            case TRANSFER:
                robotHardware.OuttakeArmServoRight.setPosition(ArmRightTransferPos);
                robotHardware.OuttakeArmServoLeft.setPosition(ArmLeftTransferPos);
                break;
            case SCORE:
                robotHardware.OuttakeArmServoRight.setPosition(ArmRightScorePos);
                robotHardware.OuttakeArmServoLeft.setPosition(ArmLeftScorePos);
                break;
            case SCORE_UP:
                robotHardware.OuttakeArmServoRight.setPosition(ArmRightScoreUpPos);
                robotHardware.OuttakeArmServoLeft.setPosition(ArmLeftScoreUpPos);
                break;
        }
    }
   public void PivotState(PivotState state) { // this is gonna be hard to control - but have a flip button but we can work it out
        switch (state) {
            case READY:
                robotHardware.PivotServo.setPosition(PivotReadyPos);
                break;
            case DIAGONAL_LEFT:
                robotHardware.PivotServo.setPosition(PivotDiagonalLeftPos);
                break;
            case DIAGONAL_RIGHT:
                robotHardware.PivotServo.setPosition(PivotDiagonalRightPos);
                break;
            case DIAGONAL_LEFT_FLIPPED:
                robotHardware.PivotServo.setPosition(PivotDiagonalLeftFlippedPos);
                break;
            case DIAGONAL_RIGHT_FLIPPED:
                robotHardware.PivotServo.setPosition(PivotDiagonalRightFlippedPos);
                break;
            case SIDEWAYS_LEFT:
                robotHardware.PivotServo.setPosition(PivotSidewaysLeftPos);
                break;
            case SIDEWAYS_RIGHT:
                robotHardware.PivotServo.setPosition(PivotSidewaysRightPos);
                break;
        }
    }

    public void wristState(WristState state) {
        switch (state) {
            case READY:
                robotHardware.WristServo.setPosition(WristReadyPos);
                break;
            case TRANSFER:
                robotHardware.WristServo.setPosition(WristTransferPos);
                break;
            case SCORE:
                robotHardware.WristServo.setPosition(WristScorePos);
                break;
        }
    }

    public void clawState(ClawState state) {
        switch (state) {
            case CLOSE:
                robotHardware.ClawServo.setPosition(ClawClosedPos);
                break;
            case OPEN:
                robotHardware.ClawServo.setPosition(ClawOpenPos);
                break;
        }
    }


}
