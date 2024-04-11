package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.MiddleLaneYDeposit;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.headingPosition;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.poseEstimate;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.xPosition;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.yPosition;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import android.graphics.Path;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.hardware.CameraHardware;
import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.StorePose;

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
    boolean goBackForYellowPixel;
    int trussMiddleStage;
    Telemetry telemetry;
    Gamepad gamepad1;
    HardwareMap hwMap;


    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    AutoTrajectories autoTrajectories = new AutoTrajectories(); // road drive class
    CameraHardware cameraHardware = new CameraHardware();

    public AutoSequences(Telemetry telemetry, int trussMiddleStage){
        this.telemetry = telemetry;
        this.trussMiddleStage = trussMiddleStage;
    }

    public void initAutoHardware (HardwareMap hardwareMap, LinearOpMode opMode){
        hwMap = hardwareMap;
        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        autoTrajectories.init(hardwareMap, opMode);

        cameraHardware.initWebcam(hardwareMap, telemetry);
        intakeSubsystem.initIntake(hardwareMap);
        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.intakeHardwareSetup();
        outtakeSubsystem.hardwareSetup();
    }

    public void intializationLoop (boolean intakeArmTop){
        //cameraHardware.pauseBackWebcam();
        outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
        if (intakeArmTop){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
        } else {
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
            telemetry.addLine("ArmToBase");
        }
        //outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
        outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition);
        outtakeSubsystem.setOuttakeRailServo(RAIL_CENTER_POS);
        if (gamepad1.dpad_left)
        {
            place = Globals.Place.LEFT;
        } else if (gamepad1.dpad_right)
        {
            place = Place.RIGHT;
        }
        telemetry.addData("Yellow Location:", cameraHardware.getPreloadYellowPose());
    }

    public void afterWaitForStart( boolean pausePreloadProcessor, Pose2d startPose){
        cameraHardware.closeWebcam(); // reduces loop times
        cameraHardware.initBackWebcamVP(hwMap, telemetry);

        GlobalTimer = new ElapsedTime(System.nanoTime());
        autoTrajectories.drive.setPoseEstimate(startPose);
        if (pausePreloadProcessor){
            cameraHardware.pausePreloadProcessor();
        } else {
            cameraHardware.resumePreloadProcessor();
        }
        //cameraHardware.resumeBackWebcam();
        //cameraHardware.resumePreloadProcessor();
        GlobalTimer.reset();
        autoTimer = GlobalTimer.milliseconds();

        goToPark = true;

        outtakeSubsystem.encodersReset();
        intakeSubsystem.intakeSlideMotorEncodersReset();
        outtakeSubsystem.cacheInitialPitchValue();

        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop
        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY); // don't touch this at all in auto
        autoTimer = 0;
        numCycles = 0;
        outtakeSubsystem.outtakeDistanceSensorValue = 100; // so that it doesn't do funky stuff

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
        yPosition = poseEstimate.getY();
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
    public boolean preloadDriveState (boolean railLeftOrRight, boolean preDropPurple, double railDelay, double slideExtendSpeed, boolean preExtendSlides){
        outtakeSubsystem.liftToInternalPID(3.9,1); // so rail doesn't hit
        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop
        intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
        if (Math.abs(yPosition) < 38){
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
                    outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.FRONT_STAGE_PURPLE);
                }
            } else if (teamPropLocation == 2){
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
            } else if (teamPropLocation == 3){
                if(railLeftOrRight){
                    outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.BACK_PURPLE);
                } else {
                    outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.BACK_STAGE_PURPLE);
                }
            }
        }

        if (autoTrajectories.preExtendSlides && preExtendSlides){
            intakeSubsystem.intakeSpin(1);
            if (trussMiddleStage == 3){
                preExtendIntakeSlidesStage(slideExtendSpeed,0);
            }
            else{
                preExtendIntakeSlides(slideExtendSpeed,0);
            }
        }
        if (autoTrajectories.dropPurple && preDropPurple){
            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
            outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
        }
        if (!autoTrajectories.drive.isBusy()){

            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
            outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
            resetTimer();
            return true;
        }
        return false;
    }
    public boolean preloadDriveStateBack(){
        outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
        if (delay(300)) {
            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE);
        }
        intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
        if (autoTrajectories.extendSlidesForBackAutos){
            resetTimer();
            return true;
        }
        return false;
    }

    public boolean placeAndIntakeFrontMIDTRUSS(double delayTimeForIntake, double slideExtendSpeed, boolean reExtendSlides){
        intakeSubsystem.intakeSpin(1);
        if (trussMiddleStage == 3){
            preExtendIntakeSlidesStage(slideExtendSpeed,0);
        }
         else{
            preExtendIntakeSlides(slideExtendSpeed,0);
        }
        if (intakeSubsystem.intakeSlideTargetReached() || limitSwitches()){ // limit switches don't touch in this case
            if (delay(delayTimeForIntake)){ // ensure pixels are in robot
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
                if(reExtendSlides && !intakeSubsystem.pixelsInIntake()){
                    intakeSubsystem.intakeSlideInternalPID(0,1);
                    goBackForYellowPixel = true;
                }
                resetTimer();
                return true;
            }
        } else {
            resetTimer(); // spams timer reset - sneaky trick
            intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
        }
        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
        outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
        if (outtakeSubsystem.pitchPosition > 23 && globalTimer-autoTimer>60){
            outtakeSubsystem.liftToInternalPID(-0.2,1);
        }
        return false;
    }

    public boolean placeAndIntakeBackSTage(double slideExtendSpeed){
        if (teamPropLocation == 1){
            intakeSubsystem.intakeSlideInternalPID(710,slideExtendSpeed);
            outtakeSubsystem.liftToInternalPID(13.9,0.9);
        } else if (teamPropLocation == 2){
            intakeSubsystem.intakeSlideInternalPID(470,slideExtendSpeed); // 265 previously
            outtakeSubsystem.liftToInternalPID(13.9,0.9);
        } else if (teamPropLocation == 3){
            intakeSubsystem.intakeSlideInternalPID(150,slideExtendSpeed);
            outtakeSubsystem.liftToInternalPID(13.9,0.9);
        }

        if (ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) > 4.2){
            outtakeSubsystem.pitchToInternalPID(19,1);
        }
        double railOffset;
        double railTarget = cameraHardware.getRailTarget(correctedHeading, ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition), outtakeSubsystem.pitchEncoderPosition);
        if (teamPropLocation == 1){
            railOffset = inchesToTicksRailCorrected(0.6);
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_RIGHT);
        } else{
            railOffset = inchesToTicksRailCorrected(-0.6);
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_LEFT);
        }

        outtakeSubsystem.setOuttakeRailServo(railTarget + railOffset);

        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE);
        outtakeSubsystem.miniTurretPointToBackdrop(correctedHeading);

        if (intakeSubsystem.intakeSlideTargetReached() && ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) > 13){ // limit switches don't touch in this case
            if (delay(100)){ // ensure pixels are in robot
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP);
                if (delay(200)){
                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                    resetTimer();
                    numCycles += 1;
                    return true;
                }
            }
        } else {
            resetTimer();
        }
        return false;
    }
    public boolean retractIntakeSlidesAfterBackstage(){
        intakeSubsystem.intakeSlideInternalPID(0,1);{
            if (intakeSubsystem.intakeSlidePosition < 100){
                return true;
            }
        }
        return false;
    }

    public boolean reExtendSLidesForYellow(double delayTimeForSlides, double slideExtendSpeed){

        if (delay(250)){ // limit switches don't touch in this case
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
            if (trussMiddleStage == 3){
                preExtendIntakeSlidesStage(slideExtendSpeed,80);
            }
            else{
                preExtendIntakeSlides(slideExtendSpeed,80);
            }

            intakeSubsystem.intakeSpin(1);
            if (delay(delayTimeForSlides)){
                //if (delay(delayTimeForIntake)){ // ensure pixels are in robot
                    intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
                    return true;
               // }
            } else {
                //resetTimer();
            }
        } else {
            intakeSubsystem.intakeSpin(-0.8);
            intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
        }
        return false;
    }

    public boolean transferPixel(){
        parkIfStuck(5500);
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
        liftDown(0.1);
        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);

        intakeSubsystem.intakeSlideInternalPID(-20,1);
        if (intakeSubsystem.intakeSlidePosition < 8){
            intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING);
        }
        if ((numCycles == 0? delay(390): delay(220)) && intakeSubsystem.intakeSlidePosition < 550){ // time for pixel holder to close/reverse
            telemetry.addLine("WE DIDN'T REVERSE THE FUCKING INTAKE BOYS");
            intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP);
            // goes back into the stack
            //goBackToStack();
            intakeSubsystem.intakeSpin(0.8);
            if ((intakeSubsystem.intakeSlidePosition < 6 || (intakeSubsystem.intakeSlidePosition < 8 && intakeSubsystem.chuteDetectorLimitSwitchValue))  && ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) < 0.1 ){
               // if (numCycles == 0? delay(1600):true){
                    intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.TRANSFER);
                    intakeSubsystem.intakeSlideInternalPID(5,1); // power draw reasons
                    resetTimer();
            //    }
                return true;
            }

        } else {
          //  if (delay(0)){
            telemetry.addLine("REVERSE THE FUCKING INTAKE");
            if (numCycles == 0){
                intakeSubsystem.intakeSpin(-0.7); // this doesn't happen for very long
            } else {
                intakeSubsystem.intakeSpin(-1); // this doesn't happen for very long
            }
           //} else {
//                intakeSubsystem.intakeSpin(1); // this doesn't happen for very long
//            }
        }
        return false;
    }

    public boolean outtakePixel(double correctedHeading, double liftTarget, int pitchTarget, int intakeSlideTarget, AutoRail autoRail,
    AutoPivot autoPivot, boolean extendStraightAway, boolean distanceSensorForCycleZero, boolean differentPitchForYellowRail, boolean openGrippers){

        if (extendStraightAway || autoTrajectories.extendSlidesAroundTruss){
            if (numCycles == 0 && !autoTrajectories.extendSlidesAroundTruss? delay(500):true){
                if (numCycles == 0 && differentPitchForYellowRail){ // consistent across all autos
                    outtakeSubsystem.setOuttakePitchYellowPixelPosition();
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.YELLOW); // slides go out before arm so transfer is good
                } else {
                    outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition);
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE); // slides go out before arm so transfer is good
                }
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
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


                    if (delay(350)){ // once chute is down
                        outtakeSubsystem.miniTurretPointToBackdrop(correctedHeading);
                        autoRail.railLogic();
                        autoPivot.pivotLogic();
                        if (!autoTrajectories.drive.isBusy() || ((outtakeSubsystem.outtakeDistanceSensorValue < OUTTAKE_DISTANCE_AUTO_THRESHOLD) && (distanceSensorForCycleZero || numCycles != 0) && ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) > (liftTarget - 4))) {
                            // line above determines when we drop the pixels
                            if (openGrippers){
                                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                            }
                            outtakeSubsystem.outtakeDistanceSensorValue = 100; // so that it doesn't do funky stuff
                            numCycles += 1;
                            timesIntoStack = 0;
                            resetTimer();
                            intakeSubsystem.intakeSpin(0);
                            autoTrajectories.extendSlidesAroundTruss = false;
                            // this can be paused after first cycle
                            cameraHardware.pausePreloadProcessor();
                            cameraHardware.pauseRailProcessor();
                            return true;
                        }
                    }

                }
                //    }
                if (delay(400)){
                    intakeSubsystem.intakeSpin(0);
                } else {
                    intakeSubsystem.intakeSpin(-0.8);
                }
            }

        } else {
            resetTimer();
        }
        return false; // returns false at the end of the method
    }

    public boolean drop(int armHeight, boolean retractLift, double delayTime){
        // waits for robot to move away far enough - minimize this
        if (delay(50)){

            //outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
            if (delay(delayTime)){ // not knock pixels off the backdrop
                if (retractLift){
                    outtakeSubsystem.liftToInternalPID(-0.7,1);
                }
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

    public boolean grabOffStack(int numCyclesForSideways, boolean switchingLanes, boolean extendSlides, int numCyclesForTurnIntoStacks, int intakeSlidePosition, double delayBeforeRetracting){
        goBackToStack = false;
        parkIfStuck(6000);

        if (delay(delayBeforeRetracting)){
            outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
            liftDown(0.08);
            outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS, 1);
            intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
            if (delay(150 + delayBeforeRetracting)){
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
            }
            // retracting arm depending on where the rail was at
            if (numCycles > numCyclesForSideways){ // wait for the outtake rail to retract
                if (delay(100 + delayBeforeRetracting)){
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                    outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                }
            } else {
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
            }
        }


        // if we are switching lanes we don't extend the slides for one cycle
        if (!extendSlides){
            telemetry.addLine("WE DO NOT WANT TO EXTEND OUR SLIDES HERE");
        } else if (switchingLanes){
            if (numCycles == (numCyclesForSideways + 1)){// don't extend slides for one cycle
                if (xPosition < 14) { // not for the last case??
                    intakeSubsystem.intakeSlideInternalPID(INTAKE_SLIDE_AUTO_LONG_PRESET, 1);
                }
            } else if (numCycles >= numCyclesForTurnIntoStacks){
                if (autoTrajectories.extendSlidesAroundStage){
                    intakeSubsystem.intakeSlideInternalPID(INTAKE_SLIDE_AUTO_LONG_PRESET,1);
                }
            } else {
                intakeSubsystem.intakeSlideInternalPID(INTAKE_SLIDE_AUTO_LONG_PRESET,1);
            }
        } else {
            if (numCycles >= numCyclesForTurnIntoStacks){
                if (autoTrajectories.extendSlidesAroundStage){
                    intakeSubsystem.intakeSlideInternalPID(INTAKE_SLIDE_AUTO_LONG_PRESET,1);
                }
            } else {
                intakeSubsystem.intakeSlideInternalPID(INTAKE_SLIDE_AUTO_LONG_PRESET,1);
            }
        }


        if (!extendSlides? autoTrajectories.extendSlidesAroundStage: xPosition < 12) { // for some reason we can't extend the slides unless we are further in
            intakeSubsystem.intakeSpin(1);
            intakeSubsystem.intakeSlideInternalPID(intakeSlidePosition,1);
            if (!autoTrajectories.drive.isBusy() || intakeSubsystem.leftArmLimitSwitchValue || intakeSubsystem.rightArmLimitSwitchValue
                    || (xPosition < -24 && intakeSubsystem.pixelsInIntake())){ // do stuff with sensor to make better
                resetTimer();
                autoTrajectories.extendSlidesAroundStage = false;
                return true;
            }
        }

        if (intakeSubsystem.pixelsInIntake() && timesIntoStack !=0){ //this isn't true unless timesIntoStack has happened at least once
            resetTimer();
            autoTrajectories.extendSlidesAroundStage = false;
            return true;
        }
        return false;
    }
    public boolean grabOffStackTruss(double delayBeforeRetracting){
        goBackToStack = false;
        //parkIfStuck(5000);
        if (delay(delayBeforeRetracting)){
            outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
            liftDown(0.08);
            outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS, 1);
            intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
            if (delay(150 + delayBeforeRetracting)){
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
            }
            if (delay(400 + delayBeforeRetracting)){
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
            }
        }

        // retracting arm depending on where the rail was at




        // if we are switching lanes we don't extend the slides for one cycle

        if (autoTrajectories.extendSlidesAroundTruss){ // not for the last case??
            intakeSubsystem.intakeSlideInternalPID(numCycles == 1?770:numCycles == 2?815:970,1);
         // for some reason we can't extend the slides unless we are further in
            intakeSubsystem.intakeSpin(1);
            if (!autoTrajectories.drive.isBusy()
                    || (xPosition < -27 && intakeSubsystem.pixelsInIntake())){ // do stuff with sensor to make better
                resetTimer();
                autoTrajectories.extendSlidesAroundTruss = false;
                autoTrajectories.extendSlidesAroundStage = false;
                return true;
            }

        } else {
            intakeSubsystem.intakeSlideInternalPID(0,1);
        }

        if (intakeSubsystem.pixelsInIntake() && timesIntoStack !=0){ //this isn't true unless timesIntoStack has happened at least once
            resetTimer();
            autoTrajectories.extendSlidesAroundTruss = false;
            autoTrajectories.extendSlidesAroundStage = false;
            return true;
        }
        return false;
    }

    public boolean reExtendSlidesForTrussSide(){
        if (numCycles > 2){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.FOUR);
        } else {
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
        }
        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
        int slideOffset = 110;
        if(delay(430)){
            intakeSubsystem.intakeSlideInternalPID(numCycles == 1?770 + slideOffset:numCycles == 2?815 + slideOffset:960 + slideOffset,0.6);
            intakeSubsystem.intakeSpin(1);
            if(intakeSubsystem.pixelsInIntake()){
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
            }
            if (intakeSubsystem.intakeSlideTargetReached() || delay(1050)){
                resetTimer();
                //intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
                return true;
            }
        } else {
            intakeSubsystem.intakeSlideInternalPID(470,1);
            intakeSubsystem.intakeSpin(-0.6);
            intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
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
        cameraHardware.closeBackWebcam();
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

    public void preExtendIntakeSlides(double slideExtendSpeed, int slideOffset){
        if (teamPropLocation == 1){
            intakeSubsystem.intakeSlideInternalPID(24 + slideOffset,slideExtendSpeed);
        } else if (teamPropLocation == 2){
            intakeSubsystem.intakeSlideInternalPID(280 + slideOffset,slideExtendSpeed); // 265 previously
        } else if (teamPropLocation == 3){
            intakeSubsystem.intakeSlideInternalPID(372 + slideOffset,slideExtendSpeed);
        }
    }
    public void preExtendIntakeSlidesStage(double slideExtendSpeed, int slideOffset){
        if (teamPropLocation == 1){
            intakeSubsystem.intakeSlideInternalPID(55 + slideOffset,slideExtendSpeed);
        } else if (teamPropLocation == 2){
            intakeSubsystem.intakeSlideInternalPID(290 + slideOffset,slideExtendSpeed); // 265 previously
        } else if (teamPropLocation == 3){
            intakeSubsystem.intakeSlideInternalPID(372 + slideOffset,slideExtendSpeed);
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
    public void goBackToStack(int trussMiddleStage, double slowerStackVelocity, double xPosition){
        if ((delay(300)) && timesIntoStack < 1 && numCycles != 0){ // go back into stack formore pixels case
            if (!intakeSubsystem.pixelsInIntake()){
                goBackToStack = true;
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
                timesIntoStack += 1;
                resetTimer();
                if (numCycles == 3){
                    autoTrajectories.goBackToStack(poseEstimate,slowerStackVelocity,trussMiddleStage,xPosition);
                }
                else {
                    autoTrajectories.goBackToStack(poseEstimate,slowerStackVelocity,trussMiddleStage,xPosition);
                }
            }
        }
    }
    public void parkIfStuck(double delayTime){
        if (delay(delayTime)){
            parkIfStuck = true;
        }
    }
    public void resetPosWithAprilTags(int tagBeingUsed){
        if (cameraHardware.getNewPose2(poseEstimate, tagBeingUsed, telemetry))
        {
            Pose2d pose = cameraHardware.getNewPose();
           /* newX = pose.getX();
            newY = pose.getY();*/
            autoTrajectories.drive.setPoseEstimate(pose);
            //cameraHardware.pauseBackWebcam();

        }
    }

    public void startAprilTagCamera(){
        cameraHardware.resumeBackWebcam();
    }
    public void storePoseEndAuto(Pose2d pose)
    {
        StorePose.pose = pose;
    }

    public void setGamepad1(Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
    }
}



