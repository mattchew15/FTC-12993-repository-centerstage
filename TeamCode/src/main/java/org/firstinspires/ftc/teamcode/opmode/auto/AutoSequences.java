package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.MiddleLaneYDeposit;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.headingPosition;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.poseEstimate;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.xPosition;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.hardware.CameraHardware;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@Config
public class AutoSequences {

    ElapsedTime GlobalTimer;
    double autoTimer;

    //int numCycles;
    double correctedHeading;
    int timesIntoStack;
    boolean goToPark;
    boolean parkIfStuck;
    boolean goToParkAfterOuttaking;
    boolean goBackToStack;
    double globalTimer;
    Telemetry telemetry;



    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    AutoTrajectories autoTrajectories = new AutoTrajectories(); // road drive class
    CameraHardware cameraHardware = new CameraHardware();

    public AutoSequences(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void initAutoHardware (HardwareMap hardwareMap, LinearOpMode opMode){
        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        autoTrajectories.init(hardwareMap, opMode);
        cameraHardware.initWebcam(hardwareMap);
    }

    public void intializationLoop (){
        outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
        intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
        //outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
        outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition);
        outtakeSubsystem.setOuttakeRailServo(RAIL_CENTER_POS);
    }

    public void afterWaitForStart(){

        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        autoTimer = GlobalTimer.milliseconds();

        goToPark = true;

        intakeSubsystem.intakeHardwareSetup();
        outtakeSubsystem.hardwareSetup();
        outtakeSubsystem.encodersReset();
        intakeSubsystem.intakeSlideMotorEncodersReset();
        outtakeSubsystem.cacheInitialPitchValue();

        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop
        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY); // don't touch this at all in auto
        autoTimer = 0;
        numCycles = 0;
        outtakeSubsystem.outtakeDistanceSensorValue = 100; // so that it doesn't do funky stuff
        autoTrajectories.drive.setPoseEstimate(autoTrajectories.startPoseFront);
        cameraHardware.closeWebcam(); // reduces loop times
    }
    public void mainAutoLoop(boolean outtakeReads, boolean intakeReads, boolean notPitchingStates){
        outtakeSubsystem.outtakeReads(outtakeReads); // might need to change this
        intakeSubsystem.intakeReads(intakeReads);
        poseEstimate = autoTrajectories.drive.getPoseEstimate();
        globalTimer = GlobalTimer.milliseconds();
        if (notPitchingStates){
            outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition);

        }
        xPosition = poseEstimate.getX();
        headingPosition = poseEstimate.getHeading();
        correctedHeading = angleWrap(Math.toDegrees(headingPosition) - 180);
        autoTrajectories.drive.update();
    }

    public boolean goToPark(boolean notStates, int wallOrMiddle){
        if (((globalTimer > 28500) && goToPark && !notStates) || parkIfStuck){
            goToPark = false;
            autoTrajectories.park(poseEstimate,wallOrMiddle);
            return true;
            }
        return false;
        }

    public boolean delayState(double delayTime){
        outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.UPRIGHT);
        outtakeSubsystem.pitchToInternalPID(PITCH_PURPLE_PIXEL_POSITION,1);
        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
        outtakeSubsystem.miniTurretPointToBackdrop(correctedHeading);
        outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition);
        intakeSubsystem.intakeSlideInternalPID(0,1);
        if (delay(delayTime)){
            resetTimer();
            return true;
        }
        return false;

    }
    public boolean preloadDriveState (boolean railLeftOrRight){
        outtakeSubsystem.liftToInternalPID(3.9,1); // so rail doesn't hit
        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop
        intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
        if (delay(1000)){
            if (railLeftOrRight){
                outtakeSubsystem.outtakeRailState(S != 1? OuttakeSubsystem.OuttakeRailState.RIGHT: OuttakeSubsystem.OuttakeRailState.LEFT);
            } else {
                outtakeSubsystem.outtakeRailState(S != 1? OuttakeSubsystem.OuttakeRailState.LEFT: OuttakeSubsystem.OuttakeRailState.RIGHT);
            }
            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_PURPLE);
            outtakeSubsystem.setOuttakePitchPurplePixelPosition();
            // turret deposit logic
            if (teamPropLocation == 1){
                if(railLeftOrRight){
                    outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.FRONT_PURPLE);
                } else {
                    outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.BACK_PURPLE);
                }
            } else if (teamPropLocation == 2){
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
            } else if (teamPropLocation == 3){
                if(railLeftOrRight){
                    outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.BACK_PURPLE);
                } else {
                    outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.FRONT_PURPLE);
                }
            }
            // pre spin intake or smth
            if (delay(2000)){
                intakeSubsystem.intakeSpin(1);
            }
        }

        if (autoTrajectories.preExtendSlides){
            intakeSubsystem.intakeSpin(1);
            preExtendIntakeSlides();
        }
        if (autoTrajectories.dropPurple){
            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
            outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
        }
        if (!autoTrajectories.drive.isBusy()){
            outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
            resetTimer();
            return true;
        }
        return false;
    }

    public boolean placeAndIntakeFrontMIDTRUSS(double delayTimeForIntake){
        intakeSubsystem.intakeSpin(1);
        preExtendIntakeSlides();
        if (intakeSubsystem.intakeSlideTargetReached() || limitSwitches() || intakeSubsystem.pixelsInIntake()){ // limit switches don't touch in this case
            if (delay(delayTimeForIntake)){ // ensure pixels are in robot
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
                resetTimer();
                return true;
            }
        } else {
            resetTimer(); // spams timer reset - sneaky trick
            intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
        }
        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
        if (outtakeSubsystem.pitchPosition > 23 && globalTimer-autoTimer>60){
            outtakeSubsystem.liftToInternalPID(-0.2,1);
        }
        return false;
    }

    public boolean transferPixel(){
        parkIfStuck(5000);
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
        liftDown(0.1);
        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
        intakeSubsystem.intakeSlideInternalPID(-20,1);
        if (numCycles == 0? delay(300): delay(220)){ // time for pixel holder to close/reverse
            intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP);
            // goes back into the stack
            //goBackToStack();

            intakeSubsystem.intakeSpin(0.8);
            if ((intakeSubsystem.intakeSlidePosition < 6 || (intakeSubsystem.intakeSlidePosition < 8 && intakeSubsystem.chuteDetectorLimitSwitchValue))  && ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) < 0.1 ){
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.TRANSFER);
                intakeSubsystem.intakeSlideInternalPID(5,1); // power draw reasons
                resetTimer();
                return true;
            }

        } else {
            if (delay(10)){
                if (numCycles == 0){
                    intakeSubsystem.intakeSpin(-0.6); // this doesn't happen for very long
                } else {
                    intakeSubsystem.intakeSpin(-1); // this doesn't happen for very long
                }
            } else {
                intakeSubsystem.intakeSpin(1); // this doesn't happen for very long
            }
        }
        return false;
    }

    public boolean outtakePixel(double correctedHeading, double liftTarget, int pitchTarget, int intakeSlideTarget, AutoRail autoRail, AutoPivot autoPivot, boolean extendStraightAway){
        if (extendStraightAway || autoTrajectories.extendSlidesAroundTruss){
            if (numCycles != 0){ // consistent across all autos
                outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE); // slides go out before arm so transfer is good
            } else {
                outtakeSubsystem.setOuttakePitchYellowPixelPosition();
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.YELLOW); // slides go out before arm so transfer is good
            }
            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
            if (delay(150)){ // time for grippers to close
                // lift and pitch to
                outtakeSubsystem.liftToInternalPID(liftTarget,1);
                outtakeSubsystem.pitchToInternalPID(pitchTarget,1);

                if (intakeSlideTarget != 0){
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget, 0.6);
                } else {
                    intakeSubsystem.intakeSlideInternalPID(intakeSlideTarget, 1);
                }
                // ready the chute after slides have gone out
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);

                if (delay(350)){
                    autoRail.railLogic();
                    if (delay(350)){ // once chute is down
                        outtakeSubsystem.miniTurretPointToBackdrop(correctedHeading);
                        autoPivot.pivotLogic();
                        if (!autoTrajectories.drive.isBusy() || (outtakeSubsystem.outtakeDistanceSensorValue < OUTTAKE_DISTANCE_AUTO_THRESHOLD && numCycles != 0)) {
                            // line above determines when we drop the pixels
                            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                            outtakeSubsystem.outtakeDistanceSensorValue = 100; // so that it doesn't do funky stuff
                            numCycles += 1;
                            timesIntoStack = 0;
                            resetTimer();
                            intakeSubsystem.intakeSpin(0);
                            autoTrajectories.extendSlidesAroundTruss = false;
                            return true;
                        }
                    }
                }
            }
            //    }
            if (delay(400)){
                intakeSubsystem.intakeSpin(0);
            } else {
                intakeSubsystem.intakeSpin(-0.8);
            }
        } else {
            resetTimer();
        }
        return false; // returns false at the end of the method
    }

    public boolean drop(int armHeight, boolean retractLift){
        // waits for robot to move away far enough - minimize this
        if (delay(50)){
            if (retractLift){
                outtakeSubsystem.liftToInternalPID(-0.6,1);
            }
            //outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
            if (delay(110)){ // not knock pixels off the backdrop
                //intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                armLogic(armHeight);
                resetTimer();
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
                return true;
            }
        }
        return false;
    }

    public void armLogic(int armHeight){
        if (armHeight == 1){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
        } else if (armHeight == 3){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE);
        } else if (armHeight == 4){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.FOUR);
        } else if (armHeight == 5){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP);
        } else if (armHeight == 6){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
        }
    }

    public boolean grabOffStack(int numCyclesForSideways, boolean switchingLanes, boolean extendSlides, int numCyclesForTurnIntoStacks){
        goBackToStack = false;
        parkIfStuck(5000);
        outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
        liftDown(0.08);
        outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS, 1);
        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
        if (delay(150)){
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
        }

        // retracting arm depending on where the rail was at
        if (numCycles > numCyclesForSideways){ // wait for the outtake rail to retract
            if (delay(600)){
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
            }
        } else {
            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
            outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
        }

        // if we are switching lanes we don't extend the slides for one cycle
        if (!extendSlides){
            telemetry.addLine("WE DO NOT WANT TO EXTEND OUR SLIDES HERE");
        } else if (switchingLanes){
            if (numCycles != (numCyclesForSideways + 1)){// don't extend slides for one cycle
                intakeSubsystem.intakeSlideInternalPID(INTAKE_SLIDE_AUTO_LONG_PRESET,1);
            } else if (numCycles >= numCyclesForTurnIntoStacks){
                if (autoTrajectories.extendSlidesAroundStage){
                    intakeSubsystem.intakeSlideInternalPID(INTAKE_SLIDE_AUTO_LONG_PRESET,1);
                }
            }
        // if we aren't switching lanes at all we extend every time :)
        } else {
            if (numCycles >= numCyclesForTurnIntoStacks){
                if (autoTrajectories.extendSlidesAroundStage){
                    intakeSubsystem.intakeSlideInternalPID(INTAKE_SLIDE_AUTO_LONG_PRESET,1);
                }
            } else {
                intakeSubsystem.intakeSlideInternalPID(INTAKE_SLIDE_AUTO_LONG_PRESET,1);
                telemetry.addLine("we are extending the slides");
            }
        }

        if (xPosition < 14){ // not for the last case??
            intakeSubsystem.intakeSlideInternalPID(INTAKE_SLIDE_AUTO_LONG_PRESET,1);
            if (xPosition < 12) { // for some reason we can't extend the slides unless we are further in
                intakeSubsystem.intakeSpin(1);
                if (!autoTrajectories.drive.isBusy() || intakeSubsystem.leftArmLimitSwitchValue || intakeSubsystem.rightArmLimitSwitchValue
                        || (xPosition < -24 && intakeSubsystem.pixelsInIntake())){ // do stuff with sensor to make better
                    resetTimer();
                    return true;
                }
            }
        }

        if (intakeSubsystem.pixelsInIntake() && timesIntoStack !=0){ //this isn't true unless timesIntoStack has happened at least once
            resetTimer();
            return true;
        }
        return false;
    }
    public boolean grabOffStackTruss(){
        goBackToStack = false;
        parkIfStuck(5000);
        double delayforFirstCycle = 0;
        if (numCycles == 1){
            delayforFirstCycle = 330;
        }
        if (delay(delayforFirstCycle)){
            outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
            liftDown(0.08);
            outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS, 1);
            intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
            if (delay(150 + delayforFirstCycle)){
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
            }
        }

        // retracting arm depending on where the rail was at

            if (delay(600)){
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
            }


        // if we are switching lanes we don't extend the slides for one cycle

        if (autoTrajectories.extendSlidesAroundTruss){ // not for the last case??
            intakeSubsystem.intakeSlideInternalPID(numCycles == 1?763:815,1);
         // for some reason we can't extend the slides unless we are further in
            intakeSubsystem.intakeSpin(1);
            if (!autoTrajectories.drive.isBusy()
                    || (xPosition < -24 && intakeSubsystem.pixelsInIntake())){ // do stuff with sensor to make better
                resetTimer();
                autoTrajectories.extendSlidesAroundTruss = false;
                return true;
            }

        } else {
            intakeSubsystem.intakeSlideInternalPID(0,1);

        }

        if (intakeSubsystem.pixelsInIntake() && timesIntoStack !=0){ //this isn't true unless timesIntoStack has happened at least once
            resetTimer();
            return true;
        }
        return false;
    }

    public boolean afterGrabOffStack(int numCyclesDropsToBase, int cycleThresholdForLongerIntake, double longerDelayTime, double shorterDelayTime){

       /*
        if (numCycles == 1 || numCycles == 3) {
            //intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.FOUR);
        }
         */
        if (numCycles == numCyclesDropsToBase){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
        }
        //TODO abstract all of the intakearm logic to its own method
        if (numCycles == numCyclesDropsToBase || (numCycles >= cycleThresholdForLongerIntake? delay(longerDelayTime) : delay(shorterDelayTime))){
            resetTimer(); // resets timer
            intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
            return true;
        }
        return false;
    }

    public boolean park(){
        if (delay(200)){
            resetTimer(); // resets timer
            return true;
        }
        return false;
    }

    public void idle(){
        intakeClipHoldorNotHold(-100);
        liftDown(0.1);
        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
        outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS, 1);
        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
        if (delay(400)){
            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
        }
        outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
    }


    public void liftDown(double liftTheshold){
        if (ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) < liftTheshold){ // if the lift is wher0e we want it
            outtakeSubsystem.liftToInternalPID(-0.05,1);
        } else {
            outtakeSubsystem.liftToInternalPID(-3,1);
        }
    }

    public boolean limitSwitches(){
        return intakeSubsystem.leftArmLimitSwitchValue || intakeSubsystem.rightArmLimitSwitchValue;
    }

    public boolean delay(double delayTime){
        return globalTimer - autoTimer > delayTime;
    }
    public void resetTimer(){
        autoTimer = globalTimer;
    }

    public void preExtendIntakeSlides(){
        if (teamPropLocation == 1){
            intakeSubsystem.intakeSlideInternalPID(30,0.9);
        } else if (teamPropLocation == 2){
            intakeSubsystem.intakeSlideInternalPID(265,0.9);
        } else if (teamPropLocation == 3){
            intakeSubsystem.intakeSlideInternalPID(300,0.9);
        }
    }
    public void intakeClipHoldorNotHold(int slideToPosition){
        if (intakeSubsystem.intakeSlidePosition < 4) {
            if (delay(40)){
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING); // turn the intake slide pid running to pos off to save battery draw
                intakeSubsystem.intakeSlideMotorRawControl(0);
            } else {
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING); // turn the intake slide pid running to pos off to save battery draw
//                intakeSubsystem.intakeSlideTo(slideToPosition,intakeSubsystem.intakeSlidePosition,1);
                intakeSubsystem.intakeSlideInternalPID(slideToPosition,1);
            }
        } else {
            intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // this might break something when as the intake slides won't go in, but stops jittering
            intakeSubsystem.intakeSlideInternalPID(slideToPosition,1);
            resetTimer();
        }
    }
    public void goBackToStack(){
        if ((delay(450)) && timesIntoStack < 1 && numCycles != 0){ // go back into stack formore pixels case
            if (!intakeSubsystem.pixelsInIntake()){
                goBackToStack = true;
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
                timesIntoStack += 1;
                resetTimer();
                if (numCycles == 3){
                    autoTrajectories.goBackToStack(poseEstimate,6,3);
                }
                else {
                    autoTrajectories.goBackToStack(poseEstimate,6,2);
                }
            }
        }
    }
    public void parkIfStuck(double delayTime){
        if (delay(delayTime)){
            parkIfStuck = true;
        }
    }
}



