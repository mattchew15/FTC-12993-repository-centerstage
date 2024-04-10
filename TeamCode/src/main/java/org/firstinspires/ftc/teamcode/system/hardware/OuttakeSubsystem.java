package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.EPSILON_DELTA;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_CENTER_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_CENTER_YELLOW_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_CENTER_YELLOW_STAGE_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_CENTER_YELLOW_TRUSS_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_LEFT_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_LEFT_YELLOW_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_LEFT_YELLOW_STAGE_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_LEFT_YELLOW_TRUSS_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_RIGHT_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_RIGHT_YELLOW_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_RIGHT_YELLOW_STAGE_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_RIGHT_YELLOW_TRUSS_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_SERVO_POSITION;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.degreestoTicksPitchMotor;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.expHub;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.inchesToTicksSlidesMotor;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.motorCaching;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.servoCaching;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.TargetCaching;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.ticksToInchesSlidesMotor;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.system.accessory.PID;
import org.firstinspires.ftc.teamcode.system.accessory.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.system.accessory.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.system.accessory.profile.ProfileSubsystem;
import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;


@Config
@SuppressWarnings("unused")
public class OuttakeSubsystem
{

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

    public static double
            MINI_TURRET_STRAIGHT_POS = 0.53,
            MINI_TURRET_FRONTPURPLE_STAGE_POS = MINI_TURRET_STRAIGHT_POS - 0.07,
            MINI_TURRET_BACKPURPLE_STAGE_POS = MINI_TURRET_STRAIGHT_POS + 0.15,
            MINI_TURRET_FRONTPURPLE_POS = MINI_TURRET_STRAIGHT_POS + 0.095,
            MINI_TURRET_BACKPURPLE_POS = MINI_TURRET_STRAIGHT_POS - 0.068;

    public static double
            ARM_READY_POS = 0.809,
            ARM_UPRIGHT_POS = 0.55,
            ARM_SCORE_POS = 0.175,
            ARM_SCORE_PURPLE_PIXEL_POS = 0.146,
            ARM_SCORE_YELLOW_POS = 0.24;
    public static double
            PIVOT_READY_POS = 0.486,
            PIVOT_DIAGONAL_LEFT_POS = PIVOT_READY_POS - 0.096,
            PIVOT_DIAGONAL_RIGHT_POS = PIVOT_READY_POS + 0.096,
            PIVOT_DIAGONAL_LEFT_FLIPPED_POS = PIVOT_READY_POS + 0.486,
            PIVOT_DIAGONAL_RIGHT_FLIPPED_POS = 0,
            PIVOT_SIDEWAYS_LEFT_POS = PIVOT_READY_POS - 0.276,
            PIVOT_SIDEWAYS_RIGHT_POS = PIVOT_READY_POS + 0.276;
    public static double
            GRIPPER_TOP_GRIP_POS = 0.25,
            GRIPPER_TOP_OPEN_POS = 0.455,
            GRIPPER_BOTTOM_GRIP_POS = 0.74,
            GRIPPER_BOTTOM_OPEN_POS = 0.545;
    public static double
            PITCH_OVERCENTERED_POSITION = 0.18,
            PITCH_PURPLEPIXEL_POSITION = 0.38,
            PITCH_LOWPITCH_POSITION = 0.7,
            PITCH_YELLOWPIXEL_POSITION = 0.4;


    public static double LiftKp = 0.015, LiftKi = 0.0001, LiftKd = 0.00002, LiftIntegralSumLimit = 10, LiftKf = 0;
    public static double LiftFTCLIBKp = 0.015, LiftFTCLIBKi = 0.0001, LiftFTCLIBKd = 0.05;

    public static double PitchKp = 0.25, PitchKi = 0.001, PitchKd = 0.001, PitchIntegralSumLimit = 5, PitchFeedforward = 0.3;
    public static double PitchFTCLIBKp = 0.14, PitchFTCLIBKi = 0.01, PitchFTCLIBKd = 0.01;

    //public static double PitchKpExternal = 0.007, PitchKiExternal = 0.000, PitchKdExternal = 0.0002, PitchIntegralSumLimitExternal = 1, PitchFeedforwardExternal = 0.3;


    PID liftPID = new PID(LiftKp,LiftKi,LiftKd,LiftIntegralSumLimit,LiftKf);
    public PID pitchPID = new PID(PitchKp,PitchKi,PitchKd,PitchIntegralSumLimit,0);
    public PIDController pitchFTCLibPID = new PIDController(PitchFTCLIBKp,PitchFTCLIBKi,PitchFTCLIBKd);
    public PIDController liftFTCLibPID = new PIDController(LiftFTCLIBKp,LiftFTCLIBKi,LiftFTCLIBKd);

    final double PITCH_THRESHOLD_DISTANCE = degreestoTicksPitchMotor(2); // could change this to a number in ticks
    final double LIFT_THRESHOLD_DISTANCE = inchesToTicksSlidesMotor(0.4);

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

    private TimedSupplier<Double> distanceSensorSupplier;

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
    private static final double RAIL_RANGE = 14.027;
    private double prevTop, prevBot, prevRail, prevMiniTurret, prevArm, prevPivot, prevPitch;
    public int prevPitchTarget, prevLiftTarget;
    private DcMotor.RunMode prevLiftMode;
    private boolean firstCyclePitch = true, firstCycleLift = true;

    public enum MiniTurretState {
        STRAIGHT,
        POINT_TO_BACKDROP,
        BACK_PURPLE,
        FRONT_PURPLE,
        FRONT_STAGE_PURPLE,
        BACK_STAGE_PURPLE
    }
    public enum OuttakeRailState {
        CENTER,
        RIGHT,
        RIGHT_YELLOW,
        RIGHT_YELLOW_TRUSS,
        RIGHT_YELLOW_STAGE,
        LEFT,
        LEFT_YELLOW,
        LEFT_YELLOW_TRUSS,
        LEFT_YELLOW_STAGE,
        FINE_ADJUST,
        CENTER_YELLOW,
        CENTER_YELLOW_TRUSS,
        CENTER_YELLOW_STAGE
    }
    public enum ArmServoState {
        READY,
        SCORE,
        CALCULATE,
        NULL,
        SCORE_PURPLE,
        UPRIGHT,
        YELLOW
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
        LiftMotor = hwMap.get(DcMotorImplEx.class, "LiftMotor");
        PitchMotor = hwMap.get(DcMotorImplEx.class, "PitchMotor");

        OuttakePitchServo =  hwMap.get(ServoImplEx.class,"OuttakePitchS");
        MiniTurretServo =  hwMap.get(ServoImplEx.class,"MiniTurretS");
        OuttakeRailServo =  hwMap.get(ServoImplEx.class,"OuttakeRailS");
        OuttakeArmServo = hwMap.get(ServoImplEx.class, "OuttakeArmS");
        PivotServo =  hwMap.get(ServoImplEx.class, "PivotS");
        GripperTopServo =  hwMap.get(ServoImplEx.class, "GripperTopS");
        GripperBottomServo =  hwMap.get(ServoImplEx.class, "GripperBottomS");
        OuttakeDistanceSensor = hwMap.get(DistanceSensor.class, "OuttakeDistanceSensor");
        PitchEncoder = hwMap.get(AnalogInput.class, "PitchEncoder");

        //distanceSensorSupplier = new TimedSupplier<>(() -> OuttakeDistanceSensor.getDistance(DistanceUnit.CM), 200);
    }

    public void hardwareSetup(){
        // We should soon decide on a final use so no run modes writes are done
        PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // run without encoder is if using external PID
        PitchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       // LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void cacheInitialPitchValue(){ // this should store the ticks required for motor to run to 0 degrees
        PitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // resets pitch motor once
        pitchTicksInitialOffset = degreestoTicksPitchMotor((getPitchEncoderPos() *0.389921)-21.4+1);
        // should store the ticks required for the pitch motor to run to zero
    }

    public void outtakeReads(boolean dropReadyState){ // pass in the drop ready state so its not reading the whole time
        liftPosition =  -LiftMotor.getCurrentPosition();
        pitchPosition = PitchMotor.getCurrentPosition();
        pitchEncoderPosition = (getPitchEncoderPos()*0.389921)-21.4+1; // offset and stuff covered here

        if (dropReadyState){ // troublesome i2c reads - we want to not call these every loop
            //outtakeDistanceSensorValue = distanceSensorSupplier.get();
            outtakeDistanceSensorValue = OuttakeDistanceSensor.getDistance(DistanceUnit.CM);
        }
    }
/*
    public void outtakeReads(boolean dropReadyState, boolean bulkEXP){ // pass in the drop ready state so its not reading the whole time
        double rawPitchEncoderPosition = getPitchEncoderPos(); // this value is only used in the following calculations, and SHOULD NOT be read twice
        liftPosition =  ticksToInchesSlidesMotor(-LiftMotor.getCurrentPosition());
        pitchEncoderPosition = (rawPitchEncoderPosition *0.389921)-21.4+1; // offset and stuff covered here
        //pitchPosition = pitchTicksInitialOffset + -PitchMotor.getCurrentPosition();


        if (dropReadyState){ // troublesome i2c reads - we want to not call these every loop
            outtakeDistanceSensorValue = OuttakeDistanceSensor.getDistance(DistanceUnit.CM);
        }

        if (bulkEXP)
        {
            expHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        else
        {
            expHub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }

    }
 */

    public double getPitchEncoderPos(){ // does work just needs to plugged in correctly
        return PitchEncoder.getVoltage() / 3.3 * 360;
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
        //prevLiftOutput = motorCaching(-manualControlLift, prevLiftOutput, EPSILON_DELTA, LiftMotor);
        LiftMotor.setPower(manualControlLift * 0.85);
    }

    public void pitchMotorRawControl(double manualControlTurret){
        PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        prevPitchOutput = motorCaching(manualControlTurret * 0.4, prevPitchOutput, EPSILON_DELTA, PitchMotor);
        //PitchMotor.setPower(manualControlTurret * -0.4);
    }

    public void liftTo(double inches, double motorPosition, double maxSpeed){
        liftTarget = (int)inchesToTicksSlidesMotor(inches);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this is added so that the external pids could be used
        double output = liftPID.update(liftTarget,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        //LiftMotor.setPower(-output);
        prevLiftOutput = motorCaching(-output, prevLiftOutput, EPSILON_DELTA, LiftMotor);
    }
    public void liftToFTCLib(int inches)
    {
        liftTarget = (int)inchesToTicksSlidesMotor(inches);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this is added so that the external pids could be used
        double output = liftFTCLibPID.calculate(liftPosition,liftTarget); //does a lift to with external PID instead of just regular encoders
        //LiftMotor.setPower(-output);
        prevLiftOutput = motorCaching(-output, prevLiftOutput, EPSILON_DELTA, LiftMotor);
    }


    /*
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

     */


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

    public void liftToInternalPID(double inches, double maxSpeed){
        liftTarget = (int)inchesToTicksSlidesMotor(inches);
        prevLiftTarget = TargetCaching(-liftTarget, prevLiftTarget, 0, LiftMotor, firstCycleLift);
        if (firstCycleLift) firstCycleLift = false;
        //LiftMotor.setTargetPosition(-liftTarget);
        //prevLiftMode = runModeCaching(DcMotor.RunMode.RUN_TO_POSITION, prevLiftMode, LiftMotor);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // should be cached by the sdk
        //prevLiftOutput = motorCaching(maxSpeed, prevLiftOutput, EPSILON_DELTA, LiftMotor);
        LiftMotor.setPower(maxSpeed);
    }
    public void liftToInternalPIDTicks(int target, double maxSpeed)
    {
        liftTarget = target;
        LiftMotor.setTargetPosition(liftTarget);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        prevLiftOutput = motorCaching(maxSpeed, prevLiftOutput, EPSILON_DELTA, LiftMotor);
        //LiftMotor.setPower(maxSpeed);
    }

    public void pitchTo(int targetDegrees, double motorPosition, double maxSpeed){
        pitchTarget = targetDegrees;
        PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double output = pitchPID.update(targetDegrees,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        prevPitchOutput = motorCaching(-output, prevPitchOutput, EPSILON_DELTA, PitchMotor);
        //PitchMotor.setPower(-output)
    }


    public void pitchToInternalPID(int degrees, double maxSpeed){
        pitchTarget = (int)(-degreestoTicksPitchMotor(degrees) + pitchTicksInitialOffset);
        prevPitchTarget = TargetCaching(pitchTarget, prevPitchTarget, 0, PitchMotor, firstCyclePitch);
        if (firstCyclePitch) firstCyclePitch = false;
        //PitchMotor.setTargetPosition(pitchTarget);
        PitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //prevPitchOutput = motorCaching(maxSpeed, prevPitchOutput, EPSILON_DELTA, PitchMotor);
        PitchMotor.setPower(maxSpeed);
    }
    public void pitchToFTCLib(int degrees)
    {
        pitchTarget = degrees;
        PitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double output = pitchFTCLibPID.calculate(pitchEncoderPosition, degrees); //does a lift to with external PID instead of just regular encoders
        prevPitchOutput = motorCaching(-output, prevPitchOutput, EPSILON_DELTA, PitchMotor);
    }


    public boolean liftTargetReached() // Simple version than the above one, should improve loop times
    {
        return (Math.abs(inchesToTicksSlidesMotor(liftTarget) - liftPosition) < LIFT_THRESHOLD_DISTANCE);
    }


    public boolean pitchTargetReached() // Simple version than the above one, should improve loop times
    {
        return (Math.abs(pitchTarget - pitchPosition) < PITCH_THRESHOLD_DISTANCE);
    }

    public static double degreesToTicksMiniTurret(double degrees){
        return MINI_TURRET_STRAIGHT_POS - degrees/350; // this should return a servoposition for the miniturret if you pass in the degrees of the robot
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

    public void outtakePitchServoKeepToPitch (double pitchAngle){ // this assumes 0 is lowest pitch and 1 is higher pitch
        double pitchServoDegrees = (Math.acos((pitchAngle-40.6238)/21.7053)/0.0147273)+5.98901;
        // we dont have to worry about boundaries because the pitch has limits
        double servoPosition = PITCH_OVERCENTERED_POSITION+degreesToTicksPitchServo(pitchServoDegrees);
        /*
        if (servoPosition > 0.15){
            servoPositionForOverCentered = servoPosition;
        } else {
            servoPositionForOverCentered = 0.15;
        }

         */
        servoPositionForOverCentered = Math.max(servoPosition, 0.15);
        prevPitch = servoCaching(servoPositionForOverCentered, prevPitch, EPSILON_DELTA, OuttakePitchServo);
    }

    public void setOuttakePitchPurplePixelPosition(){
        prevPitch = servoCaching(PITCH_PURPLEPIXEL_POSITION, prevPitch, EPSILON_DELTA, OuttakePitchServo);
        //OuttakePitchServo.setPosition(PITCH_PURPLEPIXEL_POSITION);
    }
    public void setOuttakePitchYellowPixelPosition(){
        prevPitch = servoCaching(PITCH_YELLOWPIXEL_POSITION, prevPitch, EPSILON_DELTA, OuttakePitchServo);
    }

    public void setOuttakePitchPitchDownPosition(){
        prevPitch = servoCaching(PITCH_LOWPITCH_POSITION, prevPitch, EPSILON_DELTA, OuttakePitchServo);
        //OuttakePitchServo.setPosition(PITCH_LOWPITCH_POSITION);
    }
    public void miniTurretState(MiniTurretState state) { // set this last parameter to null if not being used, R: If you do this you will raise a NullPointerException, make a default case instead... or a IDLE
        double miniTurret = 0;
        switch (state) {
            case STRAIGHT:
                miniTurret = MINI_TURRET_STRAIGHT_POS;
                //MiniTurretServo.setPosition(MINI_TURRET_STRAIGHT_POS);
                break;
            case FRONT_PURPLE:
                miniTurret = MINI_TURRET_FRONTPURPLE_POS;
                //MiniTurretServo.setPosition(MINI_TURRET_FRONTPURPLE_POS);
                break;
            case BACK_PURPLE:
                miniTurret = MINI_TURRET_BACKPURPLE_POS;
                //MiniTurretServo.setPosition(MINI_TURRET_BACKPURPLE_POS);
                break;
            case BACK_STAGE_PURPLE:
                miniTurret = MINI_TURRET_BACKPURPLE_STAGE_POS;
                break;
            case FRONT_STAGE_PURPLE:
                miniTurret = MINI_TURRET_FRONTPURPLE_STAGE_POS;
                break;
        }
        prevMiniTurret = servoCaching(miniTurret, prevMiniTurret, EPSILON_DELTA, MiniTurretServo);
    }

    public void miniTurretPointToBackdrop(double robotDegrees){
        if (Math.abs(robotDegrees) < 63){ // this is the max range of the miniturret as defined by the physical hardstops
            prevMiniTurret = servoCaching(degreesToTicksMiniTurret(robotDegrees), prevMiniTurret, EPSILON_DELTA, MiniTurretServo);
            //MiniTurretServo.setPosition(degreesToTicksMiniTurret(robotDegrees));
        }
    }

    public void outtakeRailState(OuttakeRailState state) { // set this last parameter to null if not being used, R: If you do this you will raise a NullPointerException, make a default case instead... or a IDLE
        double rail = 0;
        switch (state) {
            case CENTER:
                rail = RAIL_CENTER_POS;
                //setOuttakeRailServo(RAIL_CENTER_POS);
                break;
            case CENTER_YELLOW:
                rail = RAIL_CENTER_YELLOW_POS;
                break;
            case CENTER_YELLOW_TRUSS:
                rail = RAIL_CENTER_YELLOW_TRUSS_POS;
                break;
            case CENTER_YELLOW_STAGE:
                rail = RAIL_CENTER_YELLOW_STAGE_POS;
                break;
            case RIGHT_YELLOW:
                rail = RAIL_RIGHT_YELLOW_POS;
                break;
            case RIGHT_YELLOW_STAGE:
                rail = RAIL_RIGHT_YELLOW_STAGE_POS;
                break;
            case RIGHT_YELLOW_TRUSS:
                rail = RAIL_RIGHT_YELLOW_TRUSS_POS;
                break;
            case RIGHT:
                rail = RAIL_RIGHT_POS;
                break;
            case LEFT:
                rail = RAIL_LEFT_POS;
                break;
            case LEFT_YELLOW:
                rail = RAIL_LEFT_YELLOW_POS;
                break;
            case LEFT_YELLOW_STAGE:
                rail = RAIL_LEFT_YELLOW_STAGE_POS;
                break;
            case LEFT_YELLOW_TRUSS:
                rail = RAIL_LEFT_YELLOW_TRUSS_POS;
                break;
        }
        prevRail = servoCaching(rail, prevRail, EPSILON_DELTA, OuttakeRailServo);
    }

    public void setOuttakeRailServo(double position){
        prevRail = servoCaching(position, prevRail, EPSILON_DELTA, OuttakeRailServo);
        //OuttakeRailServo.setPosition(position);
    }

    public void fineAdjustRail(double fineAdjust, double timer){ // this is for tinkos controls only
        if (timer - outtakeRailAdjustTimer > 15){ // only set the position every 15 ms, once achieved cache the timer value
            RAIL_SERVO_POSITION += fineAdjust * 0.03; // changes global variale at .05 per 15ms
            if (RAIL_SERVO_POSITION > 1){
                RAIL_SERVO_POSITION = 1;
            } else if (RAIL_SERVO_POSITION < 0){
                RAIL_SERVO_POSITION = 0;
            }
            outtakeRailAdjustTimer = timer; // cache the value of the outtakerailadjust
        }
        // this should make the fine adjust not looptime dependent. can tune by adjusting iteration & move amount
    }

    public void armServoState(ArmServoState state) {
        double arm = 0;
        switch (state) {
            case READY:
                arm = ARM_READY_POS;
                //OuttakeArmServo.setPosition(ARM_READY_POS);
                break;
            case SCORE:
                arm = ARM_SCORE_POS;
                //OuttakeArmServo.setPosition(ARM_SCORE_POS);
                break;
            case SCORE_PURPLE:
                arm = ARM_SCORE_PURPLE_PIXEL_POS;
                //OuttakeArmServo.setPosition(ARM_SCORE_PURPLE_PIXEL_POS);
                break;
            case UPRIGHT:
                arm = ARM_UPRIGHT_POS;
                //OuttakeArmServo.setPosition(ARM_UPRIGHT_POS);
                break;
            case YELLOW:
                arm = ARM_SCORE_YELLOW_POS;
        }
        if (arm != 0){
            prevArm = servoCaching(arm, prevArm, EPSILON_DELTA, OuttakeArmServo);
        }
    }


   public void pivotServoState(PivotServoState state) { // this is gonna be hard to control - but have a flip button but we can work it out
        double pivot = 0;
        switch (state) {
            case READY:
                pivot = PIVOT_READY_POS;
                //PivotServo.setPosition(PIVOT_READY_POS);
                break;
            case DIAGONAL_LEFT:
                pivot = PIVOT_DIAGONAL_LEFT_POS;
                //PivotServo.setPosition(PIVOT_DIAGONAL_LEFT_POS);
                break;
            case DIAGONAL_RIGHT:
                pivot = PIVOT_DIAGONAL_RIGHT_POS;
                //PivotServo.setPosition(PIVOT_DIAGONAL_RIGHT_POS);
                break;
            case DIAGONAL_LEFT_FLIPPED:
                pivot = PIVOT_DIAGONAL_LEFT_FLIPPED_POS;
                //PivotServo.setPosition(PIVOT_DIAGONAL_LEFT_FLIPPED_POS);
                break;
            case DIAGONAL_RIGHT_FLIPPED:
                pivot = PIVOT_DIAGONAL_RIGHT_FLIPPED_POS;
                //PivotServo.setPosition(PIVOT_DIAGONAL_RIGHT_FLIPPED_POS);
                break;
            case SIDEWAYS_LEFT:
                pivot = PIVOT_SIDEWAYS_LEFT_POS;
                //PivotServo.setPosition(PIVOT_SIDEWAYS_LEFT_POS);
                break;
            case SIDEWAYS_RIGHT:
                pivot = PIVOT_SIDEWAYS_RIGHT_POS;
                //PivotServo.setPosition(PIVOT_SIDEWAYS_RIGHT_POS);
                break;
        }
            prevPivot = servoCaching(pivot, prevPivot, EPSILON_DELTA, PivotServo);
    }

    public void gripperServoState(GripperServoState state) {
        double top = 0, bot = 0;
        switch (state) {
            case GRIP:
                top = GRIPPER_TOP_GRIP_POS;
                bot = GRIPPER_BOTTOM_GRIP_POS;
             //   GripperTopServo.setPosition(GRIPPER_TOP_GRIP_POS);
             //   GripperBottomServo.setPosition(GRIPPER_BOTTOM_GRIP_POS);
                break;
            case OPEN:
                top = GRIPPER_TOP_OPEN_POS;
                bot = GRIPPER_BOTTOM_OPEN_POS;
            //    GripperTopServo.setPosition(GRIPPER_TOP_OPEN_POS);
            //    GripperBottomServo.setPosition(GRIPPER_BOTTOM_OPEN_POS);
                break;
            case TOP_OPEN:
                top = GRIPPER_TOP_OPEN_POS;
              //  GripperTopServo.setPosition(GRIPPER_TOP_OPEN_POS);
                break;
            case BOTTOM_OPEN:
                bot = GRIPPER_BOTTOM_OPEN_POS;
               // GripperBottomServo.setPosition(GRIPPER_BOTTOM_OPEN_POS);
                break;
        }
            if (top != 0){
                prevTop = servoCaching(top, prevTop, EPSILON_DELTA, GripperTopServo);
            }
            if (bot != 0){
                prevBot = servoCaching(bot, prevBot, EPSILON_DELTA, GripperBottomServo);
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

    public static double railInchesToTicks(double inches) {
        return inches / RAIL_RANGE;
    }
    public static double railTicksToInches(double ticks) {
        return ticks * RAIL_RANGE;
    }
}