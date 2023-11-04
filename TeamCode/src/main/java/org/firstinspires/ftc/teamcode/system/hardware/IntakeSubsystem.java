package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.system.accessory.PID;
@Config
public class IntakeSubsystem {

    public DcMotorEx
            IntakeSlideMotor,
            IntakeMotor;
    public ServoImplEx
            IntakeArmServo,
            IntakeFlapServo,
            IntakeClipServo,
            IntakePixelHolderServo;
    public ColorSensor IntakeColourSensor;

    public static double
            INTAKE_ARM_TOP_POS = 0.43,
            INTAKE_ARM_MIDDLE_POS = 0.48,
            INTAKE_ARM_BASE_POS = 0.5;
    public static double
            INTAKE_FLAP_CLOSE_POS = 0.58,
            INTAKE_FLAP_CLOSEHARD_POS = 0.59,
            INTAKE_FLAP_OPEN_POS = 0.3;
    public static double
            INTAKE_CLIP_HOLDING_POS = 0.5,
            INTAKE_CLIP_OPEN_POS = 0.75;
    public static double
            INTAKE_PIXEL_HOLDER_OPEN_POS = 0.6,
            INTAKE_PIXEL_HOLDER_HOLDING_POS = 0.3;

    final double intakeSlidethresholdDistance = 20;
    final double intakeSlidethresholdDistanceNewThreshold = 4;

    // slightly more optimal to do enums - also means we will never double write
    public enum IntakeArmServoState {
        TOP,
        MIDDLE,
        BASE
    }

    public enum IntakeFlapServoState {
        CLOSE,
        OPEN,
        CLOSE_HARD
    }

    public enum IntakeClipServoState {
        HOLDING,
        OPEN
    }

    public enum IntakePixelHolderState {
        HOLDING,
        OPEN
    }

    public static double intakeSlideKp = 0.01, intakeSlideKi = 0.00, intakeSlideKd = 0.0005, intakeSlideIntegralSumLimit = 10, intakeSlideKf = 0;
    PID intakeSlidePID = new PID(intakeSlideKp,intakeSlideKi,intakeSlideKd,intakeSlideIntegralSumLimit,intakeSlideKf);

    // define slide position and target as class members - intake slide position can be stored so its only read once
    public double intakeSlidePosition;
    public int intakeSlideTarget;
    public double colourSensorValue;

    public double degreesToTicks(double degrees) { return degrees / 355; }

    public void initIntake(HardwareMap hwMap){
        IntakeSlideMotor = hwMap.get(DcMotorEx.class, "IntakeSlideMotor");
        IntakeMotor = hwMap.get(DcMotorEx.class, "IntakeMotor");

        IntakeArmServo = hwMap.get(ServoImplEx.class, "IntakeArmS");
        IntakeClipServo = hwMap.get(ServoImplEx.class,"IntakeClipS");
        IntakeFlapServo = hwMap.get(ServoImplEx.class,"IntakeFlapS");
        IntakePixelHolderServo = hwMap.get(ServoImplEx.class,"IntakePixelHolderS");
        IntakeColourSensor = hwMap.get(ColorSensor.class,"IntakeColourSensor");
    }

    public void intakeHardwareSetup(){
        IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // handles all of the reads in this class
    public void intakeReads(){
        intakeSlidePosition = -IntakeSlideMotor.getCurrentPosition();
        colourSensorValue = IntakeColourSensor.alpha();
    }

    public void intakeSlideMotorEncodersReset(){
        IntakeSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean intakeColourSensorDetect(){
        if (colourSensorValue > 200){
            return true;
        } else {
            return false;
        }
    }

    // methods should be camel caps
    public void intakeSpin(double speedDirection){
        IntakeMotor.setPower(speedDirection);
    }

    public void intakeSlideTo(int targetRotations, double motorPosition, double maxSpeed){
        intakeSlideTarget = targetRotations;
        IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double output = intakeSlidePID.update(targetRotations,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders
        IntakeSlideMotor.setPower(-output);
    }

    public void intakeSlideInternalPID(int rotations, double maxSpeed){
        intakeSlideTarget = rotations; // variable is public to this class?
        IntakeSlideMotor.setTargetPosition(intakeSlideTarget);
        IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IntakeSlideMotor.setPower(maxSpeed);
    }
    public boolean intakeSlideTargetReached(){
        if (intakeSlidePosition > (intakeSlideTarget - intakeSlidethresholdDistance) && intakeSlidePosition < (intakeSlideTarget + intakeSlidethresholdDistance)){
            return true;
        }
        else{
            return false;
        }
    }

    public void intakeSlideMotorRawControl(double manualcontrolintakeslide){ // shouldn't have to do this - will be too slow
        IntakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeSlideMotor.setPower(manualcontrolintakeslide * -0.6);
    }

    public double intakeSlideError(){
        return intakeSlidePID.returnError();
    }

    public void intakeArmServoState(IntakeArmServoState state) {
        switch (state) {
            case TOP:
                IntakeArmServo.setPosition(INTAKE_ARM_TOP_POS);
                break;
            case MIDDLE:
                IntakeArmServo.setPosition(INTAKE_ARM_MIDDLE_POS);
                break;
            case BASE:
                IntakeArmServo.setPosition(INTAKE_ARM_BASE_POS);
                break;
        }
    }

    public void intakeFlapServoState(IntakeFlapServoState state) {
        switch (state) {
            case CLOSE:
                IntakeFlapServo.setPosition(INTAKE_FLAP_CLOSE_POS);
                break;
            case OPEN:
                IntakeFlapServo.setPosition(INTAKE_FLAP_OPEN_POS);
                break;
            case CLOSE_HARD:
                IntakeFlapServo.setPosition(INTAKE_FLAP_CLOSEHARD_POS);
                break;
        }
    }

    public void intakeClipServoState(IntakeClipServoState state) {
        switch (state) {
            case HOLDING:
                IntakeClipServo.setPosition(INTAKE_CLIP_HOLDING_POS);
                break;
            case OPEN:
                IntakeClipServo.setPosition(INTAKE_CLIP_OPEN_POS);
                break;
        }
    }

    public void intakePixelHolderServoState(IntakePixelHolderState state) {
        switch (state) {
            case HOLDING:
                IntakePixelHolderServo.setPosition(INTAKE_PIXEL_HOLDER_HOLDING_POS);
                break;
            case OPEN:
                IntakePixelHolderServo.setPosition(INTAKE_PIXEL_HOLDER_OPEN_POS);
                break;
        }
    }
}
