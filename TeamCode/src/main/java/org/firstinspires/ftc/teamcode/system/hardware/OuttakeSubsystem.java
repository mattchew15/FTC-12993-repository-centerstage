package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.system.accessory.PID;

@Config
public class OuttakeSubsystem {

    public DcMotorEx
            LiftMotor,
            PitchMotor;

    public ServoImplEx
            OuttakeArmServoLeft,
            OuttakeArmServoRight,
            MiniTurretServo,
            PivotServo,
            WristServo,
            ClawServo;

    public static double
            ARM_READY_POS = 0.93,
            ARM_TRANSFER_POS = 0.9,
            ARM_SCORE_DOWN_POS = 0.22,
            ARM_SCORE_UP_POS = 0.07;
    public static double
            MINI_TURRET_STRAIGHT_POS = 0.489,
            MINI_TURRET_LEFT_DIAGONAL_POS = 0.4,
            MINI_TURRET_RIGHT_DIAGONAL_POS = 0.6;
    public static double
            PIVOT_READY_POS = 0.53,
            PIVOT_DIAGONAL_LEFT_POS = 0.657,
            PIVOT_DIAGONAL_RIGHT_POS = 0.399,
            PIVOT_DIAGONAL_LEFT_FLIPPED_POS = 0.098,
            PIVOT_DIAGONAL_RIGHT_FLIPPED_POS = 0.958,
            PIVOT_SIDEWAYS_LEFT_POS = 0.808,
            PIVOT_SIDEWAYS_RIGHT_POS = 0.25;
    public static double
            WRIST_READY_POS = 0.26,
            WRIST_TRANSFER_POS = 0.26,
            WRIST_SCORE_POS = 0.63;
    public static double
            CLAW_OPEN_POS = 0.4,
            CLAW_CLOSE_POS = 0.65;

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
        SCORE_DOWN,
        SCORE_UP
    }

    public enum TurretServoState {
        READY,
        DIAGONAL_LEFT,
        DIAGONAL_RIGHT
    }

    public enum PivotServoState {
        READY,
        DIAGONAL_LEFT,
        DIAGONAL_RIGHT,
        DIAGONAL_LEFT_FLIPPED,
        DIAGONAL_RIGHT_FLIPPED,
        SIDEWAYS_LEFT,
        SIDEWAYS_RIGHT
    }
    public enum MiniTurretState {
        STRAIGHT,
        DIAGONAL_LEFT,
        DIAGONAL_RIGHT,
        POINT_TO_BACKDROP
    }

    public enum WristServoState {
        READY,
        TRANSFER,
        SCORE
    }

    public enum ClawServoState { // might add more states here
        CLOSE,
        OPEN
    }

    public void initOuttake(HardwareMap hwMap){
        LiftMotor = hwMap.get(DcMotorEx.class, "LiftMotor");
        PitchMotor = hwMap.get(DcMotorEx.class, "PitchMotor");

        OuttakeArmServoLeft = hwMap.get(ServoImplEx.class, "ArmSLeft");
        OuttakeArmServoRight = hwMap.get(ServoImplEx.class, "ArmSRight");
        MiniTurretServo = hwMap.get(ServoImplEx.class,"TurretS");
        PivotServo = hwMap.get(ServoImplEx.class, "PivotS");
        WristServo = hwMap.get(ServoImplEx.class, "WristS");
        ClawServo = hwMap.get(ServoImplEx.class, "ClawS");
    }

    public void hardwareSetup(){
        PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // run without encoder is if using external PID

        //OuttakeArmServo.setDirection(Servo.Direction.REVERSE);
        //turretPosition = 0; // these need to be initialized on setup or you will get null error??
        //liftPosition = 0;
        //intakeSlidePosition = 0;
    }

    public void outtakeReads(){
        pitchPosition = PitchMotor.getCurrentPosition();
        liftPosition = LiftMotor.getCurrentPosition();

        //other things like distance sensors etc
    }

    public void encodersReset(){
        PitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void liftMotorRawControl(double manualcontrollift){
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this is a write that is not needed
        LiftMotor.setPower(manualcontrollift * 0.75);
    }

    public void pitchMotorRawControl(double manualcontrolturret){
        PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PitchMotor.setPower(manualcontrolturret * -0.4);
    }

    public void liftTo(int rotations, double motorPosition, double maxSpeed){
        liftTarget = rotations;
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this is added so that the external pids could be used
        double output = liftPID.update(liftTarget,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        LiftMotor.setPower(output);
    }

    public void pitchTo(double targetRotations, double motorPosition, double maxSpeed){
        pitchTarget = targetRotations;
        PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double output = pitchPID.update(targetRotations,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        PitchMotor.setPower(output);
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

    public void armServoState(ArmServoState state) {
        switch (state) {
            case READY:
                OuttakeArmServoRight.setPosition(ARM_READY_POS);
                OuttakeArmServoLeft.setPosition(ARM_READY_POS);
            break;
            case TRANSFER:
                OuttakeArmServoRight.setPosition(ARM_TRANSFER_POS);
                OuttakeArmServoLeft.setPosition(ARM_TRANSFER_POS);
                break;
            case SCORE_DOWN:
                OuttakeArmServoRight.setPosition(ARM_SCORE_DOWN_POS);
                OuttakeArmServoLeft.setPosition(ARM_SCORE_DOWN_POS);
                break;
            case SCORE_UP:
                OuttakeArmServoRight.setPosition(ARM_SCORE_UP_POS);
                OuttakeArmServoLeft.setPosition(ARM_SCORE_UP_POS);
                break;
        }
    }

    public static double degreestoTicksMiniTurret(double degrees){
        return MINI_TURRET_STRAIGHT_POS + degrees/355; // this should return a servoposition for the miniturret if you pass in the degrees of the robot
    }

    public void miniTurretState(MiniTurretState state, double robotDegrees) { // set this last parameter to null if not being used
        switch (state) {
            case STRAIGHT:
                MiniTurretServo.setPosition(MINI_TURRET_STRAIGHT_POS);
                break;
            case DIAGONAL_LEFT:
                MiniTurretServo.setPosition(MINI_TURRET_LEFT_DIAGONAL_POS);
                break;
            case DIAGONAL_RIGHT:
                MiniTurretServo.setPosition(MINI_TURRET_RIGHT_DIAGONAL_POS);
                break;
            case POINT_TO_BACKDROP:
                MiniTurretServo.setPosition(degreestoTicksMiniTurret(robotDegrees));
                break;
        }
    }

   public void pivotServoState(PivotServoState state) { // this is gonna be hard to control - but have a flip button but we can work it out
        switch (state) {
            case READY:
                PivotServo.setPosition(PIVOT_READY_POS);
                break;
            case DIAGONAL_LEFT:
                PivotServo.setPosition(PIVOT_DIAGONAL_LEFT_POS);
                break;
            case DIAGONAL_RIGHT:
                PivotServo.setPosition(PIVOT_DIAGONAL_RIGHT_POS);
                break;
            case DIAGONAL_LEFT_FLIPPED:
                PivotServo.setPosition(PIVOT_DIAGONAL_LEFT_FLIPPED_POS);
                break;
            case DIAGONAL_RIGHT_FLIPPED:
                PivotServo.setPosition(PIVOT_DIAGONAL_RIGHT_FLIPPED_POS);
                break;
            case SIDEWAYS_LEFT:
                PivotServo.setPosition(PIVOT_SIDEWAYS_LEFT_POS);
                break;
            case SIDEWAYS_RIGHT:
                PivotServo.setPosition(PIVOT_SIDEWAYS_RIGHT_POS);
                break;
        }
    }

    public void wristServoState(WristServoState state) {
        switch (state) {
            case READY:
                WristServo.setPosition(WRIST_READY_POS);
                break;
            case TRANSFER:
                WristServo.setPosition(WRIST_TRANSFER_POS);
                break;
            case SCORE:
                WristServo.setPosition(WRIST_SCORE_POS);
                break;
        }
    }

    public void clawServoState(ClawServoState state) {
        switch (state) {
            case CLOSE:
                ClawServo.setPosition(CLAW_CLOSE_POS);
                break;
            case OPEN:
                ClawServo.setPosition(CLAW_OPEN_POS);
                break;
        }
    }
}
