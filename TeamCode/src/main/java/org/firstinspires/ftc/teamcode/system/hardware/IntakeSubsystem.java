package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.EPSILON_DELTA;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.motorCaching;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.servoCaching;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.system.accessory.PID;
import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;

@Config
@SuppressWarnings("unused")
public class IntakeSubsystem
{

    public DcMotorEx
            IntakeSlideMotor,
            IntakeMotor;
    public ServoImplEx
            IntakeArmServo,
            IntakeChuteArmServo,
            IntakeClipServo,
            IntakePixelHolderServo;

    public ColorSensor
            IntakeColourSensorFront,
            IntakeColourSensorBack;

   // public AnalogInput IntakeChuteArmEncoder;

    public DigitalChannel RightArmLimitSwitch;
    public DigitalChannel LeftArmLimitSwitch;
    public DigitalChannel ChuteUpDetectorLimitSwitch;


    public static double
            INTAKE_ARM_TOP_POS = 0.49,
            INTAKE_ARM_VERY_TOP_POS = 0.35,
            INTAKE_ARM_FOUR_POS = 0.57,
            INTAKE_ARM_MIDDLE_POS = 0.68,
            INTAKE_ARM_BASE_POS = 0.91;
    public static double
            INTAKE_CHUTE_ARM_READY_POS = 0.918,

            INTAKE_CHUTE_ARM_TRANSFER_POS = 0.4,
            INTAKE_CHUTE_ARM_HALFUP_POS = INTAKE_CHUTE_ARM_TRANSFER_POS;
    public static double
            INTAKE_CLIP_HOLDING_POS = 0.74,
            INTAKE_CLIP_OPEN_POS = 0.45;

    public static double
            INTAKE_PIXEL_HOLDER_OPEN_POS = 0.255,
            INTAKE_PIXEL_HOLDER_HOLDING_POS = 0.58;

    final double intakeSlidethresholdDistance = 12;
    final double intakeSlideThresholdDistanceNewThreshold = 4;
    private double previousSpeedDirection, previousSlideMotor;
    private double prevChute, prevArm, prevClip, prevHolder;

    // slightly more optimal to do enums - also means we will never double write
    public enum IntakeSpinState {
        INTAKE,
        SPIT_OUT,
        REVERSE
    }
    public IntakeSpinState intakeSpinState;

    double intakeTimer; // annoying i have to make it a class member

    public enum IntakeArmServoState {
        TOP,
        VERY_TOP,
        FOUR,
        MIDDLE,
        BASE
    }

    public enum IntakeChuteServoState {
        READY,
        HALF_UP,
        TRANSFER
    }

    public enum IntakeClipServoState {
        HOLDING,
        OPEN
    }

    public enum IntakePixelHolderServoState {
        HOLDING,
        OPEN
    }

    public static double intakeSlideKp = 0.01, intakeSlideKi = 0.00, intakeSlideKd = 0.0005, intakeSlideIntegralSumLimit = 10, intakeSlideKf = 0;
    PID intakeSlidePID = new PID(intakeSlideKp,intakeSlideKi,intakeSlideKd,intakeSlideIntegralSumLimit,intakeSlideKf);

    // define slide position and target as class members - intake slide position can be stored so its only read once
    public double intakeSlidePosition;
    public int intakeSlideTarget;
    public double frontColourSensorValue;
    public double backColourSensorValue;
    public boolean rightArmLimitSwitchValue;
    public boolean leftArmLimitSwitchValue;
    public boolean chuteDetectorLimitSwitchValue;
    public double intakeChuteArmPosition;
    public double intakeCurrent;
    public double intakeSlideCurrent;
    public double intakeVelocity;
    public double robotVoltage;

    private TimedSupplier<Integer> frontColorSensorSupplier, backColorSensorSupplier;

    public double degreesToTicks(double degrees) { return degrees / 355; }

    public void initIntake(HardwareMap hwMap){

        IntakeSlideMotor = hwMap.get(DcMotorEx.class, "IntakeSlideMotor");
        IntakeMotor = hwMap.get(DcMotorEx.class, "IntakeMotor");

        IntakeArmServo = hwMap.get(ServoImplEx.class, "IntakeArmS");
        IntakeClipServo = hwMap.get(ServoImplEx.class,"IntakeClipS");
        IntakeChuteArmServo = hwMap.get(ServoImplEx.class,"IntakeChuteS");
        IntakePixelHolderServo = hwMap.get(ServoImplEx.class,"IntakePixelHolderS");
        IntakeColourSensorFront = hwMap.get(ColorSensor.class,"IntakeColourSensorFront");
        IntakeColourSensorBack = hwMap.get(ColorSensor.class,"IntakeColourSensorBack");
        //IntakeChuteArmEncoder = hwMap.get(AnalogInput.class, "IntakeChuteArmEncoder");

        RightArmLimitSwitch = hwMap.get(DigitalChannel.class, "RightArmLimit");
        LeftArmLimitSwitch = hwMap.get(DigitalChannel.class, "LeftArmLimit");
        ChuteUpDetectorLimitSwitch = hwMap.get(DigitalChannel.class, "ChuteLimitSwitch");

        backColorSensorSupplier = new TimedSupplier<>(() -> IntakeColourSensorBack.alpha(), 70);
        frontColorSensorSupplier = new TimedSupplier<>(() -> IntakeColourSensorFront.alpha(), 70);
    }

    public void intakeHardwareSetup(){
        IntakeSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
       // IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       //intakeSpinState = IntakeSpinState.INTAKE;
        IntakeArmServo.setDirection(Servo.Direction.REVERSE);
    }

    // handles all of the reads in this class
    public void intakeReads(boolean I2C){ // pass in the state that the colour sensors need to be read in to optimize loop times
        intakeSlidePosition = -IntakeSlideMotor.getCurrentPosition();
        //intakeChuteArmPosition = getIntakeChuteArmPos();

        if (I2C){ // pass in state

            frontColourSensorValue = frontColorSensorSupplier.get(); // could be something else
            backColourSensorValue = backColorSensorSupplier.get();
            rightArmLimitSwitchValue = !RightArmLimitSwitch.getState();
            leftArmLimitSwitchValue = !LeftArmLimitSwitch.getState();

            chuteDetectorLimitSwitchValue = !ChuteUpDetectorLimitSwitch.getState();
            //voltageSensor.getCachedVoltage(); // this will always like return the valued already used by the driver hub
            //intakeCurrent = IntakeMotor.getCurrent(CurrentUnit.AMPS);
            //intakeSlideCurrent = IntakeSlideMotor.getCurrent(CurrentUnit.AMPS);
            //intakeVelocity = IntakeMotor.getVelocity();
            //robotVoltage = voltageSensor.getVoltage();
        }
    }
    /*
    public void intakeReads(boolean intakingState, boolean bulkReadEXP){ // pass in the state that the colour sensors need to be read in to optimize loop times
        intakeSlidePosition = -IntakeSlideMotor.getCurrentPosition();
        intakeChuteArmPosition = getIntakeChuteArmPos();
        rightArmLimitSwitchValue = !RightArmLimitSwitch.getState();
        leftArmLimitSwitchValue = !LeftArmLimitSwitch.getState();
        chuteDetectorLimitSwitchValue = !ChuteUpDetectorLimitSwitch.getState();

        if (intakingState){ // pass in state

            frontColourSensorValue = IntakeColourSensorFront.alpha(); // could be something else
            backColourSensorValue = IntakeColourSensorBack.alpha();
            intakeCurrent = IntakeMotor.getCurrent(CurrentUnit.AMPS);
            intakeVelocity = IntakeMotor.getVelocity();
            robotVoltage = voltageSensor.getVoltage();
        }

        if (bulkReadEXP)
        {
            //any read especific for the exp hub
            expHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        else
        {
            expHub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }
    }
     */

    public void intakeSlideMotorEncodersReset(){
        IntakeSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean pixelsInIntake(){
        return (frontColourSensorValue > 400) && (backColourSensorValue > 400); // should work
    }
    /*
    public double getIntakeChuteArmPos(){ // does work just needs to plugged in correctly
        return IntakeChuteArmEncoder.getVoltage() / 3.3 * 360;
    }

     */

    // methods should be camel caps

   /* public void intakePixels(double timer) {
        switch (intakeSpinState) {
            case INTAKE:
                intakeSpin(1);
                break;
            case SPIT_OUT:
                intakeTimer = timer; // resets the timer so it will reverse for a little bit
                intakeSpinState = IntakeSpinState.REVERSE;
                break;
            case REVERSE:
                intakeSpin(-1);
                if (timer - intakeTimer > 50) {
                    intakeSpinState = IntakeSpinState.INTAKE;
                }
                break;
        }
    }*/


    public void intakeSpin(double speedDirection){
        //IntakeMotor.setPower(speedDirection);
        //IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        previousSpeedDirection = motorCaching(speedDirection, previousSpeedDirection, EPSILON_DELTA, IntakeMotor);
    }

    public void intakeSlideTo(int targetRotations, double motorPosition, double maxSpeed){
        intakeSlideTarget = targetRotations;
        //IntakeSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        double output = intakeSlidePID.update(targetRotations,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        previousSlideMotor = motorCaching(-output, previousSlideMotor, EPSILON_DELTA, IntakeSlideMotor);
        //IntakeSlideMotor.setPower(-output);
    }

    public void intakeSlideInternalPID(int rotations, double maxSpeed){
        intakeSlideTarget = -rotations; // variable is public to this class?
        IntakeSlideMotor.setTargetPosition(intakeSlideTarget); //TODO cache
        IntakeSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        previousSlideMotor = motorCaching(maxSpeed, previousSlideMotor, EPSILON_DELTA, IntakeSlideMotor);
        //IntakeSlideMotor.setPower(maxSpeed);
    }
    public boolean intakeSlideTargetReached(){
        //if (intakeSlidePosition > (intakeSlideTarget - intakeSlidethresholdDistance) && intakeSlidePosition < (intakeSlideTarget + intakeSlidethresholdDistance)){
        //    return true;
        //}
        //else{
        //    return false;
        //}
        return  Math.abs(-intakeSlideTarget - intakeSlidePosition) < intakeSlidethresholdDistance;
    }

    public void intakeSlideMotorRawControl(double manualcontrolintakeslide){ // shouldn't have to do this - will be too slow
        IntakeSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        previousSlideMotor = motorCaching(manualcontrolintakeslide * -0.75, previousSlideMotor, EPSILON_DELTA, IntakeSlideMotor);
        //IntakeSlideMotor.setPower(manualcontrolintakeslide * -0.6);
    }

    public double intakeSlideError(){
        return intakeSlidePID.returnError();
    }

    public void intakeArmServoState(IntakeArmServoState state) {
        switch (state) {
            case TOP:
                prevArm = servoCaching(INTAKE_ARM_TOP_POS, prevArm, EPSILON_DELTA, IntakeArmServo);
                //IntakeArmServo.setPosition(INTAKE_ARM_TOP_POS);
                break;
            case MIDDLE:
                prevArm = servoCaching(INTAKE_ARM_MIDDLE_POS, prevArm, EPSILON_DELTA, IntakeArmServo);
                //IntakeArmServo.setPosition(INTAKE_ARM_MIDDLE_POS);
                break;
            case VERY_TOP:
                prevArm = servoCaching(INTAKE_ARM_VERY_TOP_POS, prevArm, EPSILON_DELTA, IntakeArmServo);
                //IntakeArmServo.setPosition(INTAKE_ARM_VERY_TOP_POS);
                break;
            case FOUR:
                prevArm = servoCaching(INTAKE_ARM_FOUR_POS, prevArm, EPSILON_DELTA, IntakeArmServo);
                //IntakeArmServo.setPosition(INTAKE_ARM_FOUR_POS);
                break;
            case BASE:
                prevArm = servoCaching(INTAKE_ARM_BASE_POS, prevArm, EPSILON_DELTA, IntakeArmServo);
                //IntakeArmServo.setPosition(INTAKE_ARM_BASE_POS);
                break;
        }
    }

    public void intakeChuteArmServoState(IntakeChuteServoState state) {
        double chute = 0;
        switch (state) {
            case READY:
                chute = INTAKE_CHUTE_ARM_READY_POS;
                //IntakeChuteArmServo.setPosition(INTAKE_CHUTE_ARM_READY_POS);
                break;
            case HALF_UP:
                chute = INTAKE_CHUTE_ARM_HALFUP_POS;
                //IntakeChuteArmServo.setPosition(INTAKE_CHUTE_ARM_HALFUP_POS);
                break;
            case TRANSFER:
                chute = INTAKE_CHUTE_ARM_TRANSFER_POS;
                //IntakeChuteArmServo.setPosition(INTAKE_CHUTE_ARM_TRANSFER_POS);
                break;
        }
        prevChute = servoCaching(chute, prevChute, EPSILON_DELTA, IntakeChuteArmServo);
    }

    public void intakeClipServoState(IntakeClipServoState state) {
        double clip = 0;
        switch (state) {
            case HOLDING:
                clip = INTAKE_CLIP_HOLDING_POS;
                //IntakeClipServo.setPosition(INTAKE_CLIP_HOLDING_POS);
                break;
            case OPEN:
                clip = INTAKE_CLIP_OPEN_POS;
                //IntakeClipServo.setPosition(INTAKE_CLIP_OPEN_POS);
                break;
        }
        prevClip = servoCaching(clip, prevClip, EPSILON_DELTA, IntakeClipServo);
    }

    public void intakePixelHolderServoState(IntakePixelHolderServoState state) {
        double holder = 0;
        switch (state) {
            case HOLDING:
                holder = INTAKE_PIXEL_HOLDER_HOLDING_POS;
                //IntakePixelHolderServo.setPosition(INTAKE_PIXEL_HOLDER_HOLDING_POS);
                break;
            case OPEN:
                holder = INTAKE_PIXEL_HOLDER_OPEN_POS;
                //IntakePixelHolderServo.setPosition(INTAKE_PIXEL_HOLDER_OPEN_POS);
                break;
        }
        prevHolder = servoCaching(holder, prevHolder, EPSILON_DELTA, IntakePixelHolderServo);
    }
}
