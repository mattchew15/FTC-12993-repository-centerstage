package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.ticksToInchesSlidesMotor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.system.accessory.OuttakeInverseKinematics;
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
            OuttakePitchServo,
            MiniTurretServo,
            OuttakeRailServo,
            OuttakeArmServo,
            PivotServo,
            GripperTopServo,
            GripperBottomServo;

    public AnalogInput
            PitchEncoder;

    public DistanceSensor OuttakeDistanceSensor;

    public static double MINI_TURRET_STRAIGHT_POS = 0.5;
    public static double
            RAIL_CENTER_POS = 0.49,
            RAIL_RIGHT_POS = 0,
            RAIL_LEFT_POS = 1;
    public static double
            ARM_READY_POS = 0.235,
            ARM_SCORE_POS = 0.85,
            ARM_SCORE_PURPLE_PIXEL_POS = 0.86;
    public static double
            PIVOT_READY_POS = 0.489,
            PIVOT_DIAGONAL_LEFT_POS = 0.334,
            PIVOT_DIAGONAL_RIGHT_POS = 0.634,
            PIVOT_DIAGONAL_LEFT_FLIPPED_POS = 0.87,
            PIVOT_DIAGONAL_RIGHT_FLIPPED_POS = 0.06,
            PIVOT_SIDEWAYS_LEFT_POS = 0.205,
            PIVOT_SIDEWAYS_RIGHT_POS = 0.759;
    public static double
            GRIPPER_TOP_OPEN_POS = 0.24,
            GRIPPER_TOP_GRIP_POS = 0.5,
            GRIPPER_BOTTOM_OPEN_POS = 0.73,
            GRIPPER_BOTTOM_GRIP_POS = 0.5;
    public static double
            PITCH_OVERCENTERED_POSITION = 0.12;

    public static double LiftKp = 0.015, LiftKi = 0.0001, LiftKd = 0.00002, LiftIntegralSumLimit = 10, LiftKf = 0;
    public static double PitchKp = 0.25, PitchKi = 0.001, PitchKd = 0.001, PitchIntegralSumLimit = 5, PitchFeedforward = 0.3;
  //public static double PitchKpExternal = 0.007, PitchKiExternal = 0.000, PitchKdExternal = 0.0002, PitchIntegralSumLimitExternal = 1, PitchFeedforwardExternal = 0.3;


    PID liftPID = new PID(LiftKp,LiftKi,LiftKd,LiftIntegralSumLimit,LiftKf);
    public PID pitchPID = new PID(PitchKp,PitchKi,PitchKd,PitchIntegralSumLimit,0);

    final double PITCH_THRESHOLD_DISTANCE = degreestoTicksPitchMotor(2); // could change this to a number in ticks
    final double LIFT_THRESHOLD_DISTANCE = 1;

    public int pitchTarget;
    public int liftTarget;

    public double pitchPosition;
    public double liftPosition;
    public double outtakeDistanceSensorValue;
    public double outtakeRailAdjustTimer;
    public double outtakeLiftAdjustTimer;
    public double pitchEncoderPosition;
    public double pitchTicksInitialOffset;
    public double initialPitchDegrees;
    public double servoPositionForOverCentered;

    private OuttakeInverseKinematics outtakeInverseKinematics = new OuttakeInverseKinematics();

    // The profile stuff
    public double kPos = 0.0001, kVel = 0.0001;
    public final double LiftPKp = 0.0038, LiftPKi = 0.0001, LiftPKd = 0.00006, LiftPIntegralSumLimit = 10;
    public final PID PLiftPID = new PID(LiftPKp, LiftPKi, LiftPKd, LiftPIntegralSumLimit, 0);
    public static double velConstrain = 5000, accelConstrain = 4900, decelConstrain = 4800;
    public ProfileConstraints profileLiftConstraints = new ProfileConstraints(velConstrain, accelConstrain, decelConstrain);
    // Make private after tuning
    public AsymmetricMotionProfile liftProfile  = new AsymmetricMotionProfile(liftPosition, liftTarget, profileLiftConstraints);
    public ProfileSubsystem profileSubsystem = new ProfileSubsystem(PLiftPID);
    public double liftProfileStartingPosition;

    // servos profile stuff
    private double armTarget, armPos; // armPos is the previous target
    public static double armVelConstrain = 0.1, armAccelConstrain = 0.1, armDecelConstrain = 0.1;
    private ProfileConstraints profileArmConstraints = new ProfileConstraints(armVelConstrain, armAccelConstrain, armDecelConstrain);
    private AsymmetricMotionProfile armProfile;
    private ElapsedTime armProfileTimer = new ElapsedTime();
    private double prevLiftOutput;
    private double prevPitchOutput;


    public enum MiniTurretState {
        STRAIGHT,
        POINT_TO_BACKDROP
    }
    public enum OuttakeRailState {
        CENTER,
        RIGHT,
        LEFT,
        FINE_ADJUST
    }
    public enum ArmServoState {
        READY,
        SCORE,
        CALCULATE,
        NULL,
        SCORE_PURPLE
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
    public enum GripperServoState { // might add more states here
        GRIP,
        OPEN,
        TOP_OPEN,
        BOTTOM_OPEN
    }

    public enum OuttakeResetState {
        UP,
        DOWN,
        IDLE
    }
    public OuttakeResetState outtakeResetState;

    public void initOuttake(HardwareMap hwMap){
        LiftMotor = hwMap.get(DcMotorEx.class, "LiftMotor");
        PitchMotor = hwMap.get(DcMotorEx.class, "PitchMotor");

        OuttakePitchServo = hwMap.get(ServoImplEx.class,"OuttakePitchS");
        MiniTurretServo = hwMap.get(ServoImplEx.class,"MiniTurretS");
        OuttakeRailServo = hwMap.get(ServoImplEx.class,"OuttakeRailS");
        OuttakeArmServo = hwMap.get(ServoImplEx.class, "OuttakeArmS");
        PivotServo = hwMap.get(ServoImplEx.class, "PivotS");
        GripperTopServo = hwMap.get(ServoImplEx.class, "GripperTopS");
        GripperBottomServo = hwMap.get(ServoImplEx.class, "GripperBottomS");
        OuttakeDistanceSensor = hwMap.get(DistanceSensor.class, "OuttakeDistanceSensor");
        PitchEncoder = hwMap.get(AnalogInput.class, "PitchEncoder");
    }

    public void hardwareSetup(){
        PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // run without encoder is if using external PID
        PitchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       // LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void cacheInitialPitchValue(){
        initialPitchDegrees = (angleWrapDegrees(getPitchEncoderPos()) *0.389921)+54.4; // offset and stuff covered here
        PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void outtakeReads(boolean dropReadyState){ // pass in the drop ready state so its not reading the whole time
        double rawPitchEncoderPosition = getPitchEncoderPos(); // this value is only used in the following calculations, and SHOULD NOT be read twice
        liftPosition =  ticksToInchesSlidesMotor(-LiftMotor.getCurrentPosition());
        pitchEncoderPosition = (rawPitchEncoderPosition *0.389921)-21.4+1; // offset and stuff covered here
        //pitchPosition = pitchTicksInitialOffset + -PitchMotor.getCurrentPosition();


        if (dropReadyState){ // troublesome i2c reads - we want to not call these every loop
            outtakeDistanceSensorValue = OuttakeDistanceSensor.getDistance(DistanceUnit.CM);
        }
    }

    public double getPitchEncoderPos(){ // does work just needs to plugged in correctly
        return PitchEncoder.getVoltage() / 3.15 * 360;
    }

    public void encodersReset(){
        PitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void liftEncoderReset(){
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void liftMotorRawControl(double manualControlLift){
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this is a write that is not needed
        prevLiftOutput = motorCaching(-manualControlLift, prevLiftOutput, EPSILON_DELTA, LiftMotor);
        // LiftMotor.setPower(manualControlLift * -1);

    }

    public void pitchMotorRawControl(double manualControlTurret){
        PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        prevPitchOutput = motorCaching(manualControlTurret * -0.4, prevPitchOutput, EPSILON_DELTA, PitchMotor);
        //PitchMotor.setPower(manualControlTurret * -0.4);
    }

    public void liftTo(int inches, double motorPosition, double maxSpeed){
        liftTarget = (int)inchesToTicksSlidesMotor(inches);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this is added so that the external pids could be used
        double output = liftPID.update(liftTarget,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        LiftMotor.setPower(-output);
        //prevLiftOutput = motorCaching(-output, prevLiftOutput, EPSILON_DELTA, LiftMotor);

    }
    public void fineAdjustLift(double inches, double timer)
    {
        if (timer - outtakeLiftAdjustTimer > 20)
        {
            double newValue = ticksToInchesSlidesMotor(liftTarget) + inches;
            liftTarget = (int) ticksToInchesSlidesMotor(outtakeInverseKinematics.slideEnd(newValue));
            //liftTarget = Math.min(liftTarget, LIFT_INCHES_FOR_MAX_EXTENSION);
            liftToInternalPIDTicks(liftTarget, 1);
            pitchToInternalPID((int) outtakeInverseKinematics.pitchEnd(newValue), 1);
            outtakeLiftAdjustTimer = timer;
        }

    }


    public void resetOuttake() {
        switch (outtakeResetState) {
            case UP:
                liftTo(170, liftPosition, 1);
                if (liftTargetReached()){
                    outtakeResetState = OuttakeResetState.DOWN;
                }
                break;
            case DOWN:
                liftToInternalPID(-200, 1);
                if (liftPosition < 5){
                    liftToInternalPID(0, 1);
                    outtakeResetState = OuttakeResetState.IDLE;
                }
                break;
            case IDLE:

                break;
        }
    }

    public void liftToInternalPID(int inches, double maxSpeed){
        liftTarget = (int)inchesToTicksSlidesMotor(inches);
        LiftMotor.setTargetPosition(-liftTarget);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setPower(maxSpeed);
    }
    public void liftToInternalPIDTicks(int target, double maxSpeed)
    {
        liftTarget = target;
        LiftMotor.setTargetPosition(liftTarget);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //prevLiftOutput = motorCaching(maxSpeed, prevLiftOutput, EPSILON_DELTA, LiftMotor);
        LiftMotor.setPower(maxSpeed);
    }

    public void pitchTo(int targetDegrees, double motorPosition, double maxSpeed){
        pitchTarget = targetDegrees;
        PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double output = pitchPID.update(targetDegrees,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        prevPitchOutput = motorCaching(-output, prevPitchOutput, EPSILON_DELTA, PitchMotor);
        //PitchMotor.setPower(-output)
    }


    public void pitchToInternalPID(int degrees, double maxSpeed){

        pitchTarget = (int)degreestoTicksPitchMotor(initialPitchDegrees-degrees);
        PitchMotor.setTargetPosition(pitchTarget);
        PitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        prevPitchOutput = motorCaching(maxSpeed, prevPitchOutput, EPSILON_DELTA, PitchMotor);
        //PitchMotor.setPower(maxSpeed);
    }


    public boolean liftTargetReached() // Simple version than the above one, should improve loop times
    {
        return (Math.abs(liftTarget - liftPosition) < LIFT_THRESHOLD_DISTANCE);
    }


    public boolean pitchTargetReached() // Simple version than the above one, should improve loop times
    {
        return (Math.abs(pitchTarget - pitchPosition) < PITCH_THRESHOLD_DISTANCE);
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

    public static double degreesToTicksPitchServo(double degrees){
        return degrees/355; // this should return a servoposition for the miniturret if you pass in the degrees of the robot
    }

    public void outtakePitchServoKeepToPitch (double pitchAngle, Telemetry telemtry){ // this assumes 0 is lowest pitch and 1 is higher pitch
        double pitchServoDegrees = (Math.acos((pitchAngle-40.6238)/21.7053)/0.0147273)+5.98901;
        // we dont have to worry about boundaries because the pitch has limits
        double servoPosition = PITCH_OVERCENTERED_POSITION+degreesToTicksPitchServo(pitchServoDegrees);

        if (servoPosition > 0.15){
            servoPositionForOverCentered = servoPosition;
        } else {
            servoPositionForOverCentered = 0.15;
        }
        OuttakePitchServo.setPosition(servoPositionForOverCentered);
        telemtry.addData("SERVOPOSITION FOR OVERCENTERED", servoPositionForOverCentered);
    }

    public void miniTurretState(MiniTurretState state) { // set this last parameter to null if not being used, R: If you do this you will raise a NullPointerException, make a default case instead... or a IDLE
        switch (state) {
            case STRAIGHT:
                MiniTurretServo.setPosition(MINI_TURRET_STRAIGHT_POS);
                break;
        }
    }

    public void miniTurretPointToBackdrop(double robotDegrees){
        if (Math.abs(robotDegrees) < 63){ // this is the max range of the miniturret as defined by the physical hardstops
            MiniTurretServo.setPosition(degreesToTicksMiniTurret(robotDegrees));
        }
    }

    public void outtakeRailState(OuttakeRailState state) { // set this last parameter to null if not being used, R: If you do this you will raise a NullPointerException, make a default case instead... or a IDLE
        switch (state) {
            case CENTER:
                RAIL_SERVO_POSITION = RAIL_CENTER_POS;
                break;
            case RIGHT:
                RAIL_SERVO_POSITION = RAIL_RIGHT_POS;
                break;
            case LEFT:
                RAIL_SERVO_POSITION = RAIL_LEFT_POS;
                break;
        }
    }

    public void setOuttakeRailServo(double position){
        OuttakeRailServo.setPosition(position);
    }

    public void fineAdjustRail(double fineAdjust, double timer){ // this is for tinkos controls only
        if (timer - outtakeRailAdjustTimer > 15){ // only set the position every 15 ms, once achieved cache the timer value
            RAIL_SERVO_POSITION += fineAdjust * 0.03; // changes global variale at .05 per 15ms
            outtakeRailAdjustTimer = timer; // cache the value of the outtakerailadjust
        }
        // this should make the fine adjust not looptime dependent. can tune by adjusting iteration & move amount
    }

    public void armServoState(ArmServoState state) {
        switch (state) {
            case READY:
                OuttakeArmServo.setPosition(ARM_READY_POS);
                break;
            case SCORE:
                OuttakeArmServo.setPosition(ARM_SCORE_POS);
                break;
            case SCORE_PURPLE:
                OuttakeArmServo.setPosition(ARM_SCORE_PURPLE_PIXEL_POS);
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

















    // below is profile stuff
    public void rawLift(double pow)
    {
        // The manual control lift is weighted this is different.
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // not a necessary write
        prevLiftOutput = motorCaching(pow, prevLiftOutput, EPSILON_DELTA, LiftMotor);
        //LiftMotor.setPower(pow);
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
        profileSubsystem.initState(); // this might fuck up; to be complete honest this is unnecessary write that set the variables to zero, why it is here i don't know, like i don't even recall this state being accessed later
        profileSubsystem.setTarget(liftTarget);
        profileSubsystem.setPos(liftPosition); // this should be the currentPosition, this is for the PID
        double output = profileSubsystem.calculate();
        //rawLift(output);
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
        profileSubsystem.setPos(liftPosition); // this should be the currentPosition, this is for the PID
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
        OuttakeArmServo.setPosition(pos);
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


    public void armServoProfileState(ArmServoState state) {
        switch (state) {
            case READY:
                setArmTarget(ARM_READY_POS);
                break;
            case SCORE:
                setArmTarget(ARM_SCORE_POS);
                break;
            case CALCULATE:
                calculateArmProfile();
                break;
            default:
        }
        // previous version calculated always
    }

    public void armServoProfileState2(ArmServoState state, boolean update) {
        switch (state) {
            case READY:
                if (update) setArmTarget(ARM_READY_POS);
                else calculateArmProfile();
                break;
            case SCORE:
                if (update) setArmTarget(ARM_SCORE_POS);
                else calculateArmProfile();
                break;
            case CALCULATE:
                calculateArmProfile();
                break;
            default:
        }
    }



}