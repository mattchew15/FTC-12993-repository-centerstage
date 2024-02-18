package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.system.accessory.PID;

@Photon
@Config
public class IntakeSubsystemPhoton
{

    public PhotonDcMotor
            IntakeSlideMotor,
            IntakeMotor;
    public PhotonServo
            IntakeArmServo,
            IntakeChuteArmServo,
            IntakeClipServo,
            IntakePixelHolderServo;

    public ColorSensor
            IntakeColourSensorFront,
            IntakeColourSensorBack;

    public AnalogInput
            IntakeChuteArmEncoder;

    public DigitalChannel FrontLimitSwitch;
    // todo: change for photon
    VoltageSensor voltageSensor;

    public static double
            INTAKE_ARM_TOP_POS = 0.479,
            INTAKE_ARM_VERY_TOP_POS = 0.481,
            INTAKE_ARM_FOUR_POS = 0.477,
            INTAKE_ARM_MIDDLE_POS = 0.425,
            INTAKE_ARM_BASE_POS = 0.37;
    public static double
            INTAKE_CHUTE_ARM_READY_POS = 0.15,
            INTAKE_CHUTE_ARM_HALFUP_POS = 0.38,
            INTAKE_CHUTE_ARM_TRANSFER_POS = 0.8;
    public static double
            INTAKE_CLIP_HOLDING_POS = 0.58,
            INTAKE_CLIP_OPEN_POS = 0.84;
    public static double
            INTAKE_PIXEL_HOLDER_OPEN_POS = 0.42,
            INTAKE_PIXEL_HOLDER_HOLDING_POS = 0.82;

    final double intakeSlidethresholdDistance = 20;
    final double intakeSlidethresholdDistanceNewThreshold = 4;
    private double previousIntakeSlidePower;
    private double previousIntakeSpin;

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

    public enum IntakePixelHolderState {
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
    public boolean frontLimitSwitchValue;
    public double intakeChuteArmPosition;
    public double intakeCurrent;
    public double intakeVelocity;
    public double robotVoltage;

    public double degreesToTicks(double degrees) { return degrees / 355; }

    public void initIntake(HardwareMap hwMap){
        IntakeSlideMotor = hwMap.get(PhotonDcMotor.class, "IntakeSlideMotor");
        IntakeMotor = hwMap.get(PhotonDcMotor.class, "IntakeMotor");

        IntakeArmServo = hwMap.get(PhotonServo.class, "IntakeArmS");
        IntakeClipServo = hwMap.get(PhotonServo.class,"IntakeClipS");
        IntakeChuteArmServo = hwMap.get(PhotonServo.class,"IntakeChuteS");
        IntakePixelHolderServo = hwMap.get(PhotonServo.class,"IntakePixelHolderS");
        IntakeColourSensorFront = hwMap.get(ColorSensor.class,"IntakeColourSensorFront");
        IntakeColourSensorBack = hwMap.get(ColorSensor.class,"IntakeColourSensorBack");
        IntakeChuteArmEncoder = hwMap.get(AnalogInput.class, "IntakeChuteArmEncoder");
        FrontLimitSwitch = hwMap.get(DigitalChannel.class, "FrontLimitSwitch");
        voltageSensor = hwMap.voltageSensor.iterator().next();
    }

    public void intakeHardwareSetup(){
        IntakeSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeSpinState = IntakeSpinState.INTAKE;
    }

    // handles all of the reads in this class
    public void intakeReads(boolean intakingState){ // pass in the state that the colour sensors need to be read in to optimize loop times
        intakeSlidePosition = -IntakeSlideMotor.getCurrentPosition();
        intakeChuteArmPosition = getIntakeChuteArmPos();
        frontLimitSwitchValue = !FrontLimitSwitch.getState();
        if (intakingState){ // pass in state
            frontColourSensorValue = IntakeColourSensorFront.alpha(); // could be something else
            backColourSensorValue = IntakeColourSensorBack.alpha();
            intakeCurrent = IntakeMotor.getCurrent(CurrentUnit.AMPS);
            intakeVelocity = IntakeMotor.getVelocity();
            robotVoltage = voltageSensor.getVoltage();
        }
    }

    public void intakeSlideMotorEncodersReset(){
        IntakeSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean pixelsInIntake(){
        return (frontColourSensorValue > 350) && (backColourSensorValue > 2500); // should work
    }
    public double getIntakeChuteArmPos(){ // does work just needs to plugged in correctly
        return IntakeChuteArmEncoder.getVoltage() / 3.3 * 360;
    }


    // methods should be camel caps

    public void intakePixels(double timer) {
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
    }


    public void intakeSpin(double speedDirection){
        if (Math.abs(speedDirection - previousIntakeSpin) > 0.005)
        {
            IntakeSlideMotor.setPower(speedDirection);
            previousIntakeSpin = speedDirection;
        }
    }

    public void intakeSlideTo(int targetRotations, double motorPosition, double maxSpeed){
        intakeSlideTarget = targetRotations;
        IntakeSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        double output = intakeSlidePID.update(targetRotations,motorPosition,maxSpeed); //does a lift to with external PID instead of just regular encoders

        if (Math.abs(output - previousIntakeSlidePower) > 0.005)
        {
            IntakeSlideMotor.setPower(-output);
            previousIntakeSlidePower = output;
        }
    }
    public void intakeSlideInternalPID(int rotations, double maxSpeed){
        intakeSlideTarget = -rotations; // variable is public to this class?
        IntakeSlideMotor.setTargetPosition(intakeSlideTarget);
        IntakeSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (Math.abs(maxSpeed - previousIntakeSlidePower) > 0.005)
        {
            IntakeSlideMotor.setPower(maxSpeed);
            previousIntakeSlidePower = maxSpeed;
        }

    }
    public boolean intakeSlideTargetReached(){
        return  Math.abs(intakeSlideTarget - intakeSlidePosition) < intakeSlidethresholdDistance;
    }

    public void intakeSlideMotorRawControl(double manualcontrolintakeslide){ // shouldn't have to do this - will be too slow
        IntakeSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        double output = (manualcontrolintakeslide * -0.6);
        if (Math.abs(output - previousIntakeSlidePower) > 0.005)
        {
            IntakeSlideMotor.setPower(output);
            previousIntakeSlidePower = output;
        }

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
            case VERY_TOP:
                IntakeArmServo.setPosition(INTAKE_ARM_VERY_TOP_POS);
                break;
            case FOUR:
                IntakeArmServo.setPosition(INTAKE_ARM_FOUR_POS);
                break;
            case BASE:
                IntakeArmServo.setPosition(INTAKE_ARM_BASE_POS);
                break;
        }
    }

    public void intakeChuteArmServoState(IntakeChuteServoState state) {
        switch (state) {
            case READY:
                IntakeChuteArmServo.setPosition(INTAKE_CHUTE_ARM_READY_POS);
                break;
            case HALF_UP:
                IntakeChuteArmServo.setPosition(INTAKE_CHUTE_ARM_HALFUP_POS);
                break;
            case TRANSFER:
                IntakeChuteArmServo.setPosition(INTAKE_CHUTE_ARM_TRANSFER_POS);
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
