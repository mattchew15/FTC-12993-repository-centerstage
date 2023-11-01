package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.system.accessory.PID;

@Config
public class OuttakeSubsystem {

    RobotHardware robotHardware;

    public static double
            ARM_LEFT_READY_POS = 0.43,
            ARM_LEFT_TRANSFER_POS = 0.212,
            ARM_LEFT_SCORE_POS = 0,
            ARM_LEFT_SCORE_UP_POS = 0;
    public static double
            ARM_RIGHT_READY_POS = 0.43,
            ARM_RIGHT_TRANSFER_POS = 0.212,
            ARM_RIGHT_SCORE_POS = 0,
            ARM_RIGHT_SCORE_UP_POS = 0;
    public static double
            TURRET_READY_POS = 0.5,
            TURRET_DIAGONAL_LEFT_POS = 0.25,
            TURRET_DIAGONAL_RIGHT_POS = 0.75;
    public static double
            PIVOT_READY_POS = 0.145,
            PIVOT_DIAGONAL_LEFT_POS = 0.241,
            PIVOT_DIAGONAL_RIGHT_POS = 0.32,
            PIVOT_DIAGONAL_LEFT_FLIPPED_POS = 0,
            PIVOT_DIAGONAL_RIGHT_FLIPPED_POS,
            PIVOT_SIDEWAYS_LEFT_POS = 0.145,
            PIVOT_SIDEWAYS_RIGHT_POS = 0;
    public static double
            WRIST_READY_POS = 0.43,
            WRIST_TRANSFER_POS = 0.212,
            WRIST_SCORE_POS;
    public static double
            CLAW_CLOSE_POS = 0.3,
            CLAW_OPEN_POS = 0.7;
    public static double MiniTurretStraightPos = 0.7, MiniTurretLeftDiagonalPos = 0.3, MiniTurretRightDiagonalPos = 0.7;

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

    public void armServoState(ArmServoState state) {
        switch (state) {
            case READY:
                robotHardware.OuttakeArmServoRight.setPosition(ARM_RIGHT_READY_POS);
                robotHardware.OuttakeArmServoLeft.setPosition(ARM_LEFT_READY_POS);
            break;
            case TRANSFER:
                robotHardware.OuttakeArmServoRight.setPosition(ARM_RIGHT_TRANSFER_POS);
                robotHardware.OuttakeArmServoLeft.setPosition(ARM_LEFT_TRANSFER_POS);
                break;
            case SCORE:
                robotHardware.OuttakeArmServoRight.setPosition(ARM_RIGHT_SCORE_POS);
                robotHardware.OuttakeArmServoLeft.setPosition(ARM_LEFT_SCORE_POS);
                break;
            case SCORE_UP:
                robotHardware.OuttakeArmServoRight.setPosition(ARM_RIGHT_SCORE_UP_POS);
                robotHardware.OuttakeArmServoLeft.setPosition(ARM_LEFT_SCORE_UP_POS);
                break;
        }
    }

    public void turretServoState(TurretServoState state) {
        switch (state) {
            case READY:
                robotHardware.TurretServo.setPosition(TURRET_READY_POS);
                break;
            case DIAGONAL_LEFT:
                robotHardware.TurretServo.setPosition(TURRET_DIAGONAL_LEFT_POS);
                break;
            case DIAGONAL_RIGHT:
                robotHardware.TurretServo.setPosition(TURRET_DIAGONAL_RIGHT_POS);
                break;
        }
    }

   public void pivotServoState(PivotServoState state) { // this is gonna be hard to control - but have a flip button but we can work it out
        switch (state) {
            case READY:
                robotHardware.PivotServo.setPosition(PIVOT_READY_POS);
                break;
            case DIAGONAL_LEFT:
                robotHardware.PivotServo.setPosition(PIVOT_DIAGONAL_LEFT_POS);
                break;
            case DIAGONAL_RIGHT:
                robotHardware.PivotServo.setPosition(PIVOT_DIAGONAL_RIGHT_POS);
                break;
            case DIAGONAL_LEFT_FLIPPED:
                robotHardware.PivotServo.setPosition(PIVOT_DIAGONAL_LEFT_FLIPPED_POS);
                break;
            case DIAGONAL_RIGHT_FLIPPED:
                robotHardware.PivotServo.setPosition(PIVOT_DIAGONAL_RIGHT_FLIPPED_POS);
                break;
            case SIDEWAYS_LEFT:
                robotHardware.PivotServo.setPosition(PIVOT_SIDEWAYS_LEFT_POS);
                break;
            case SIDEWAYS_RIGHT:
                robotHardware.PivotServo.setPosition(PIVOT_SIDEWAYS_RIGHT_POS);
                break;
        }
    }

    public void wristServoState(WristServoState state) {
        switch (state) {
            case READY:
                robotHardware.WristServo.setPosition(WRIST_READY_POS);
                break;
            case TRANSFER:
                robotHardware.WristServo.setPosition(WRIST_TRANSFER_POS);
                break;
            case SCORE:
                robotHardware.WristServo.setPosition(WRIST_SCORE_POS);
                break;
        }
    }


    public static double degreestoTicksMiniTurret(double degrees){
        return MiniTurretStraightPos + degrees/355; // this should return a servoposition for the miniturret if you pass in the degrees of the robot
    }

    public void miniTurretState(MiniTurretState state, double robotDegrees) { // set this last parameter to null if not being used
        switch (state) {
            case STRAIGHT:
                robotHardware.MiniTurretServo.setPosition(MiniTurretStraightPos);
                break;
            case DIAGONAL_LEFT:
                robotHardware.MiniTurretServo.setPosition(MiniTurretLeftDiagonalPos);
                break;
            case DIAGONAL_RIGHT:
                robotHardware.MiniTurretServo.setPosition(MiniTurretRightDiagonalPos);
                break;
            case POINT_TO_BACKDROP:
                robotHardware.MiniTurretServo.setPosition(degreestoTicksMiniTurret(robotDegrees));
                break;
        }
    }

    public void clawServoState(ClawServoState state) {
        switch (state) {
            case CLOSE:
                robotHardware.ClawServo.setPosition(CLAW_CLOSE_POS);
                break;
            case OPEN:
                robotHardware.ClawServo.setPosition(CLAW_OPEN_POS);
                break;
        }
    }
}
