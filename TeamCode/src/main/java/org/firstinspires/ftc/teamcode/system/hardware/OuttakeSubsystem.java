package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.system.accessory.PID;
import org.firstinspires.ftc.teamcode.system.accessory.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.system.accessory.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.system.accessory.profile.ProfileSubsystem;

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
            GripperTopServo,
            GripperBottomServo;

    public DistanceSensor OuttakeDistanceSensor;

    public static double
            ARM_READY_POS = 0.93,
            ARM_TRANSFER_POS = 0.85,
            ARM_PREEXTEND_POS = 0.78,
            ARM_SCORE_DOWN_POS = 0.22,
            ARM_SCORE_UP_POS = 0.16;
    public static double
            MINI_TURRET_STRAIGHT_POS = 0.489,
            MINI_TURRET_LEFT_DIAGONAL_POS = 0.4,
            MINI_TURRET_RIGHT_DIAGONAL_POS = 0.6;
    public static double
            PIVOT_READY_POS = 0.502,
            PIVOT_DIAGONAL_LEFT_POS = 0.627,
            PIVOT_DIAGONAL_RIGHT_POS = 0.369,
            PIVOT_DIAGONAL_LEFT_FLIPPED_POS = 0.068,
            PIVOT_DIAGONAL_RIGHT_FLIPPED_POS = 0.928,
            PIVOT_SIDEWAYS_LEFT_POS = 0.778,
            PIVOT_SIDEWAYS_RIGHT_POS = 0.22;
    public static double
            GRIPPER_TOP_OPEN_POS,
            GRIPPER_TOP_GRIP_POS,
            GRIPPER_BOTTOM_OPEN_POS,
            GRIPPER_BOTTOM_GRIP_POS;

    public static double LiftKp = 0.015, LiftKi = 0.0001, LiftKd = 0.00006, LiftIntegralSumLimit = 10, LiftKf = 0;
    public static double PitchKp = 0.007, PitchKi = 0.000, PitchKd = 0.0002, PitchIntegralSumLimit = 1, PitchFeedforward = 0.3;

    PID liftPID = new PID(LiftKp,LiftKi,LiftKd,LiftIntegralSumLimit,LiftKf);
    PID pitchPID = new PID(PitchKp,PitchKi,PitchKd,PitchIntegralSumLimit,PitchFeedforward);

    final double PITCH_THRESHOLD_DISTANCE = degreestoTicksPitchMotor(2); // could change this to a number in ticks
    final double LIFT_THRESHOLD_DISTANCE = 22;

    public int pitchTarget;
    public int liftTarget;

    public double pitchPosition;
    public double liftPosition;
    public double outtakeDistanceSensorValue;

    // The profile stuff
    public static double kPos = 0.2, kVel = 0.2;
    public static double LiftPKp = 0.015, LiftPKi = 0.0001, LiftPKd = 0.00006, LiftPIntegralSumLimit = 10;
    private PID PLiftPID = new PID(LiftPKp, LiftPKi, LiftPKd, LiftPIntegralSumLimit, 0);
    public static ProfileConstraints profileSliderConstraints = new ProfileConstraints(11000, 11000, 11000);
    // TODO: make a new constructor, to pass a pid object
    private AsymmetricMotionProfile liftProfile  = new AsymmetricMotionProfile(liftPosition, liftTarget, profileSliderConstraints);
    private ProfileSubsystem profileSubsystem;

    //Servo stuff
    public enum ArmServoState {
        READY,
        TRANSFER,
        PRE_EXTEND,
        SCORE_DOWN,
        SCORE_UP
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

    public enum GripperServoState { // might add more states here
        GRIP,
        OPEN
    }

    public void initOuttake(HardwareMap hwMap){
        LiftMotor = hwMap.get(DcMotorEx.class, "LiftMotor");
        PitchMotor = hwMap.get(DcMotorEx.class, "PitchMotor");

        OuttakeArmServoLeft = hwMap.get(ServoImplEx.class, "ArmSLeft");
        OuttakeArmServoRight = hwMap.get(ServoImplEx.class, "ArmSRight");
        MiniTurretServo = hwMap.get(ServoImplEx.class,"MiniTurretS");
        PivotServo = hwMap.get(ServoImplEx.class, "PivotS");
        GripperTopServo = hwMap.get(ServoImplEx.class, "GripperTopS");
        GripperBottomServo = hwMap.get(ServoImplEx.class, "GripperBottomS");
        OuttakeDistanceSensor = hwMap.get(DistanceSensor.class, "OuttakeDistanceSensor");
    }

    public void hardwareSetup(){
        PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // run without encoder is if using external PID

        PitchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void softwareSetup()
    {
        profileSubsystem = new ProfileSubsystem(PLiftPID);
    }

    public void outtakeReads(boolean dropReadyState){ // pass in the drop ready state so its not reading the whole time
        pitchPosition = PitchMotor.getCurrentPosition(); // only reads in the whole class
        liftPosition = -LiftMotor.getCurrentPosition();
        if (dropReadyState){
            outtakeDistanceSensorValue = OuttakeDistanceSensor.getDistance(DistanceUnit.CM);
        }
    }

    public void encodersReset(){
        PitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void liftMotorRawControl(double manualControlLift){
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this is a write that is not needed
        LiftMotor.setPower(manualControlLift * 0.75);
    }

    public void pitchMotorRawControl(double manualControlTurret){
        PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PitchMotor.setPower(manualControlTurret * -0.4);
    }

    public void liftTo(int rotations, double motorPosition, double maxSpeed){
        liftTarget = rotations;
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this is added so that the external pids could be used
        double output = liftPID.update(liftTarget,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        LiftMotor.setPower(-output);
    }
    public void liftToInternalPID(int rotations, double maxSpeed){
        liftTarget = rotations;
        LiftMotor.setTargetPosition(liftTarget);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setPower(maxSpeed);
    }
    /*
    public void pitchTo(int targetRotations, double motorPosition, double maxSpeed){
        pitchTarget = targetRotations;
        PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double output = pitchPID.update(targetRotations,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        PitchMotor.setPower(output);
    }

     */
    public void pitchToInternalPID(int rotations, double maxSpeed){
        pitchTarget = rotations;
        //telemetry.addData("lifttarget", liftTarget);
        PitchMotor.setTargetPosition(pitchTarget);
        PitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        PitchMotor.setPower(maxSpeed);
    }

    public boolean liftTargetReached(){
        if (liftPosition > (liftTarget - LIFT_THRESHOLD_DISTANCE) && liftPosition < (liftTarget+ LIFT_THRESHOLD_DISTANCE)){ //LIFT_THRESHOLD_DISTANCE, simplify with Math.abs
            return true;
        }
        else{
            return false;
        }
    }
    public boolean liftTargetReachedR() // Simple version than the above one, should improve loop times
    {
        return (Math.abs(liftTarget - liftPosition) < LIFT_THRESHOLD_DISTANCE);
    }

    public boolean pitchTargetReached(){
        if (pitchPosition < (pitchTarget + PITCH_THRESHOLD_DISTANCE) && pitchPosition > (pitchTarget- PITCH_THRESHOLD_DISTANCE)){
            return true;
        }
        else{
            return false;
        }
    }
    public boolean pitchTargetReachedR() // Simple version than the above one, should improve loop times
    {
        return (Math.abs(pitchTarget - pitchPosition) < PITCH_THRESHOLD_DISTANCE);
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
            case PRE_EXTEND:
                OuttakeArmServoRight.setPosition(ARM_PREEXTEND_POS);
                OuttakeArmServoLeft.setPosition(ARM_PREEXTEND_POS);
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

    public static double degreesToTicksMiniTurret(double degrees){
        return MINI_TURRET_STRAIGHT_POS + degrees/355; // this should return a servoposition for the miniturret if you pass in the degrees of the robot
    }

    public void miniTurretState(MiniTurretState state) { // set this last parameter to null if not being used, R: If you do this you will raise a NullPointerException, make a default case instead... or a IDLE
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
                //removed this stupid shit
                break;
        }
    }

    public void miniTurretPointToBackdrop(double robotDegrees){
        MiniTurretServo.setPosition(degreesToTicksMiniTurret(robotDegrees));
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

    public void gripperServoState(GripperServoState state) {
        switch (state) {
            case GRIP:
                GripperTopServo.setPosition(GRIPPER_TOP_GRIP_POS);
                GripperBottomServo.setPosition(GRIPPER_BOTTOM_GRIP_POS);
                break;
            case OPEN:
                GripperTopServo.setPosition(GRIPPER_TOP_OPEN_POS);
                GripperBottomServo.setPosition(GRIPPER_BOTTOM_OPEN_POS);
                break;
        }
    }
    public void rawLift(double pow)
    {
        // The manual control lift is weighted this is different.
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setPower(-pow);
    }
    public void profileSetUp()
    {
        profileSubsystem.setProfile(liftProfile);
        profileSubsystem.setTolerance(LIFT_THRESHOLD_DISTANCE);
        profileSubsystem.setMaxOutput(1); // not a necessary write
    }
    public double profileCalculate()
    {
        profileSubsystem.setPos(liftPosition);
        return profileSubsystem.calculateFullState();

    }
    public void setFullStateGains(double kPos, double kVel)
    {
        OuttakeSubsystem.kPos = kPos;
        OuttakeSubsystem.kVel = kVel;
    }
}
