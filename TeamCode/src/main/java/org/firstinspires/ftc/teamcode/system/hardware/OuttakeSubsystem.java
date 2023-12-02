package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

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
            ARM_READY_POS = 0.885,
            ARM_UPRIGHT_POS = 0.4,
            ARM_SCORE_HALF_DOWN_POS = 0.1,
            ARM_SCORE_DOWN_POS = 0.18,
            ARM_SCORE_UP_POS = 0.05;
    public static double
            MINI_TURRET_STRAIGHT_POS = 0.5,
            MINI_TURRET_LEFT_DIAGONAL_POS = 0.38,
            MINI_TURRET_RIGHT_DIAGONAL_POS = 0.62;
    public static double
            PIVOT_READY_POS = 0.502,
            PIVOT_DIAGONAL_LEFT_POS = 0.627,
            PIVOT_DIAGONAL_RIGHT_POS = 0.369,
            PIVOT_DIAGONAL_LEFT_FLIPPED_POS = 0.068,
            PIVOT_DIAGONAL_RIGHT_FLIPPED_POS = 0.928,
            PIVOT_SIDEWAYS_LEFT_POS = 0.778,
            PIVOT_SIDEWAYS_RIGHT_POS = 0.22;
    public static double
            GRIPPER_TOP_OPEN_POS = 0.88,
            GRIPPER_TOP_GRIP_POS = 0.73,
            GRIPPER_BOTTOM_OPEN_POS = 0.25,
            GRIPPER_BOTTOM_GRIP_POS = 0.38;

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
    public double kPos, kVel;
    public final double LiftPKp = 0.003, LiftPKi = 0.0001, LiftPKd = 0.00006, LiftPIntegralSumLimit = 10;
    public final PID PLiftPID = new PID(LiftPKp, LiftPKi, LiftPKd, LiftPIntegralSumLimit, 0);
    public static double velConstrain = 4400, accelConstrain = 3700, decelConstrain = 4200;
    public ProfileConstraints profileLiftConstraints = new ProfileConstraints(velConstrain, accelConstrain, decelConstrain);
    // Make private after tuning
    public AsymmetricMotionProfile liftProfile  = new AsymmetricMotionProfile(liftPosition, liftTarget, profileLiftConstraints);
    public ProfileSubsystem profileSubsystem = new ProfileSubsystem(PLiftPID);
    public double liftProfileStartingPosition;

    // servos profile stuff
    private double armTarget, armPos; // armPos is the previous target
    private ProfileConstraints profileArmConstraints = new ProfileConstraints(500, 500, 500);
    private AsymmetricMotionProfile armProfile;
    private ElapsedTime armProfileTimer = new ElapsedTime();


    public enum ArmServoState {
        READY,
        UPRIGHT,
        SCORE_HALF_DOWN,
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
        OPEN,
        TOP_OPEN,
        BOTTOM_OPEN
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

    public void outtakeReads(boolean dropReadyState){ // pass in the drop ready state so its not reading the whole time
        pitchPosition = PitchMotor.getCurrentPosition(); // only reads in the whole class
        liftPosition = LiftMotor.getCurrentPosition();
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
        LiftMotor.setPower(manualControlLift * 1);
    }

    public void pitchMotorRawControl(double manualControlTurret){
        PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PitchMotor.setPower(manualControlTurret * -0.4);
    }

    public void liftTo(int rotations, double motorPosition, double maxSpeed){
        liftTarget = rotations;
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this is added so that the external pids could be used
        double output = liftPID.update(liftTarget,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        LiftMotor.setPower(output);
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

    //public boolean liftTargetReachedOld(){
    //    if (liftPosition > (liftTarget - LIFT_THRESHOLD_DISTANCE) && liftPosition < (liftTarget+ LIFT_THRESHOLD_DISTANCE)){ //LIFT_THRESHOLD_DISTANCE, simplify with Math.abs
    //        return true;
    //    }
    //    else{
    //        return false;
    //    }
    //}
    public boolean liftTargetReached() // Simple version than the above one, should improve loop times
    {
        return (Math.abs(liftTarget - liftPosition) < LIFT_THRESHOLD_DISTANCE);
    }

    //public boolean pitchTargetReachedOld(){
    //    if (pitchPosition < (pitchTarget + PITCH_THRESHOLD_DISTANCE) && pitchPosition > (pitchTarget- PITCH_THRESHOLD_DISTANCE)){
    //        return true;
    //    }
    //    else{
    //        return false;
    //    }
    //}
    public boolean pitchTargetReached() // Simple version than the above one, should improve loop times
    {
        return (Math.abs(pitchTarget - pitchPosition) < PITCH_THRESHOLD_DISTANCE);
    }

    public void armServoState(ArmServoState state) {
        switch (state) {
            case READY:
                OuttakeArmServoRight.setPosition(ARM_READY_POS);
                OuttakeArmServoLeft.setPosition(ARM_READY_POS);
            break;
            case UPRIGHT:
                OuttakeArmServoRight.setPosition(ARM_UPRIGHT_POS);
                OuttakeArmServoLeft.setPosition(ARM_UPRIGHT_POS);
                break;
            case SCORE_HALF_DOWN:
                OuttakeArmServoRight.setPosition(ARM_SCORE_HALF_DOWN_POS);
                OuttakeArmServoLeft.setPosition(ARM_SCORE_HALF_DOWN_POS);
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
    public void armServoProfileState(ArmServoState state) {
        switch (state) {
            case READY:
                setArmTarget(ARM_READY_POS);
                break;
            case UPRIGHT:
                setArmTarget(ARM_UPRIGHT_POS);
                break;
            case SCORE_HALF_DOWN:
                setArmTarget(ARM_SCORE_HALF_DOWN_POS);
                break;
            case SCORE_DOWN:
                setArmTarget(ARM_SCORE_DOWN_POS);
                break;
            case SCORE_UP:
                setArmTarget(ARM_SCORE_UP_POS);
                break;
        }
        calculateArmProfile();
    }

    public static double degreesToTicksMiniTurret(double degrees){
        return MINI_TURRET_STRAIGHT_POS - degrees/355; // this should return a servoposition for the miniturret if you pass in the degrees of the robot
    }
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
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
        if (Math.abs(robotDegrees) < 63){
            MiniTurretServo.setPosition(degreesToTicksMiniTurret(robotDegrees));
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
            case TOP_OPEN:
                GripperTopServo.setPosition(GRIPPER_TOP_OPEN_POS);
                break;
            case BOTTOM_OPEN:
                GripperBottomServo.setPosition(GRIPPER_BOTTOM_OPEN_POS);
                break;
        }
    }
    public void rawLift(double pow)
    {
        // The manual control lift is weighted this is different.
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // not a necessary write
        LiftMotor.setPower(pow);
    }
    public void profileLiftSetUp()
    {
        profileSubsystem.setProfile(liftProfile);
        profileSubsystem.setTolerance(LIFT_THRESHOLD_DISTANCE);
        profileSubsystem.setMaxOutput(1); // not a necessary write
        profileSubsystem.setFullStateFeedback(kPos, kVel);
        profileSubsystem.initState();
    }
    public double profileLiftCalculate()
    {
        liftProfile = new AsymmetricMotionProfile(liftProfileStartingPosition, liftTarget, profileLiftConstraints);
        profileSubsystem.setMode(ProfileSubsystem.SubsystemMode.FullState);
        profileSubsystem.setTolerance(LIFT_THRESHOLD_DISTANCE);
        profileSubsystem.setMaxOutput(1); // not a necessary write
        profileSubsystem.setFullStateFeedback(kPos, kVel);
        profileSubsystem.initState(); // this might fuck up
        profileSubsystem.setTarget(liftTarget);
        profileSubsystem.setPos(liftPosition); // this should be the currentPosition, this is for the PID
        double output = profileSubsystem.calculate();
        rawLift(output);
        return output;
    }
    public double profileLiftCalculateFeedForward()
    {
        profileSubsystem.setFeedforward(0.04, ProfileSubsystem.SubsystemMode.CONSTANT);
        profileSubsystem.setTolerance(LIFT_THRESHOLD_DISTANCE);
        profileSubsystem.setMaxOutput(1); // not a necessary write
        profileSubsystem.setFullStateFeedback(kPos, kVel);
        profileSubsystem.initState();
        profileSubsystem.setTarget(liftTarget);
        profileSubsystem.setPos(liftPosition); // this should be the currentPosition, this is for the PID TODO: try making this the state.x or passing the state.x as the profile pos
        double output = profileSubsystem.calculate();
        rawLift(output);
        return output;
    }
    public void setLiftFullStateGains(double kPos, double kVel)
    {
        this.kPos = kPos;
        this.kVel = kVel;
    }
    public void updateLiftTargetProfile(int liftTarget)
    {
        liftProfile = new AsymmetricMotionProfile(liftProfileStartingPosition, liftTarget, profileLiftConstraints);
        profileSubsystem.setProfile(liftProfile);
        liftProfileStartingPosition = liftPosition;
        profileSubsystem.resetTime();
        this.liftTarget = liftTarget;
    }

    public void calculateArmProfile()
    {
        armProfile = new AsymmetricMotionProfile(armTarget ,armPos , profileArmConstraints);
        armProfile.calculate(armProfileTimer.time());
        double pos = armProfile.state.x;
        OuttakeArmServoRight.setPosition(pos);
        OuttakeArmServoLeft.setPosition(pos);
    }
    public void setArmTarget(double armTarget)
    {
        // this.armPos = OuttakeArmServoLeft.getPosition();
        this.armPos = this.armTarget;
        this.armTarget = armTarget;
        armProfileTimer.reset();
    }
    public double getArmX()
    {
        return armProfile.state.x;
    }
    public double getArmPos()
    {
        return armPos;
    }


}