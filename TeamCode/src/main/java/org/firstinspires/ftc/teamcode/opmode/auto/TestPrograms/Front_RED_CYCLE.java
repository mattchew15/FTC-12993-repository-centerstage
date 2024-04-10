package org.firstinspires.ftc.teamcode.opmode.auto.TestPrograms;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories;
import org.firstinspires.ftc.teamcode.opmode.teleop.SimplicityDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.CameraHardware;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.*;

import android.provider.Settings;
import android.view.ViewTreeObserver;
@Disabled
@Autonomous(name = "Front Red Cycle Auto", group = "Autonomous")
public class Front_RED_CYCLE extends LinearOpMode {

    // class members
    ElapsedTime GlobalTimer;
    double autoTimer;

    int numCycles;
    double correctedHeading;
    int timesIntoStack;
    boolean goToPark;
    double globalTimer;

    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    CameraHardware cameraHardware = new CameraHardware();
    AutoTrajectories autoTrajectories;
    //Accessories
    LoopTime loopTime = new LoopTime();


    enum AutoState {
        DELAY,
        PRELOAD_DRIVE,
        PLACE_AND_INTAKE,
        AFTER_PURPLE_DRIVE,
        TRANSFER_PIXEL,
        OUTTAKE_PIXEL,
        DROP,
        GRAB_OFF_STACK,
        AFTER_GRAB_OFF_STACK,
        AFTER_GRAB_OFF_STACK_TOP,
        OUTTAKE_PIXEL_NO_INTAKE_SLIDES,
        PARK,
        IDLE,
    }


    AutoState currentState;



    @Override
    public void runOpMode() throws InterruptedException {

        SetAuto.setRedAuto();
        autoTrajectories = new AutoTrajectories(); // road drive class

        // initialize hardware

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            module.clearBulkCache();
        } //


        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
      //  cameraHardware.initWebcam(hardwareMap);
        autoTrajectories.init(hardwareMap, this);
        // functions runs on start


        // trajectories that aren't changing should all be here
        while (!isStarted()) { // initialization loop
            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
            //outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
            outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition);
            outtakeSubsystem.setOuttakeRailServo(RAIL_CENTER_POS);
            if (teamPropLocation == 1){
                telemetry.addLine("Front");
            } else if (teamPropLocation == 2){
                telemetry.addLine("Middle");
            } else if (teamPropLocation == 3){
                telemetry.addLine("Back");
            }
            telemetry.addData("S", S);
            telemetry.update();
        }


        waitForStart();
        if (isStopRequested()) return;
        // runs instantly once
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
       // autoTrajectories.drive.setPoseEstimate(autoTrajectories.startPoseFront);

        currentState = AutoState.DELAY;
       // cameraHardware.closeWebcam(); // reduces loop times

        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            outtakeSubsystem.outtakeReads(currentState == AutoState.OUTTAKE_PIXEL && xPosition > 24); // might need to change this
            intakeSubsystem.intakeReads(currentState == AutoState.GRAB_OFF_STACK || currentState == AutoState.AFTER_GRAB_OFF_STACK || currentState == AutoState.PLACE_AND_INTAKE);
            poseEstimate = autoTrajectories.drive.getPoseEstimate();
            globalTimer = GlobalTimer.milliseconds();

            // Print pose to telemetry

            //loopTime.updateLoopTime(telemetry);
            loopTime.delta();
            telemetry.addData("LoopTime", loopTime.getDt() / 1_000_000);
            telemetry.addData("Hz", loopTime.getHz());

            telemetry.addData("Auto State", currentState);

     //       telemetry.addData("IntakeSlideMotor Current", intakeSubsystem.IntakeSlideMotor.getCurrent(CurrentUnit.AMPS));
     //       telemetry.addData("LiftMotor Current", outtakeSubsystem.LiftMotor.getCurrent(CurrentUnit.AMPS));
     //       telemetry.addData("PitchMotor Current", outtakeSubsystem.PitchMotor.getCurrent(CurrentUnit.AMPS));

            //telemetry.addData("numCycles", numCycles);
            //telemetry.addData("distance sensor value", outtakeSubsystem.outtakeDistanceSensorValue);
            //telemetry.addData ("times into stack", timesIntoStack);
            //telemetry.addData("intakeMotorVelocity", intakeSubsystem.intakeVelocity);
            //telemetry.addData("right Limit Switch", intakeSubsystem.rightArmLimitSwitchValue);
            //telemetry.addData("left Limit Switch", intakeSubsystem.leftArmLimitSwitchValue);
            //telemetry.addData("s",S);

            //telemetry.addLine("");
            //telemetry.addData("liftPosition", ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition));
            telemetry.addData("pitch encoder Position", outtakeSubsystem.pitchEncoderPosition);
            telemetry.addData("intakeSlidePosition", intakeSubsystem.intakeSlidePosition);
            //telemetry.addData("lift Target Reached", outtakeSubsystem.liftTargetReached());
            autoSequence();
            if (currentState != AutoState.PRELOAD_DRIVE && currentState != AutoState.OUTTAKE_PIXEL){
                outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition);
            }


            xPosition = poseEstimate.getX();              // is in globals rn - might not work idk
            //yPosition = poseEstimate.getY();
            headingPosition = poseEstimate.getHeading();

            //telemetry.addData("x Position", xPosition);
            //telemetry.addData("y Position", yPosition);
            //telemetry.addData("heading", headingPosition);
            correctedHeading = angleWrap(Math.toDegrees(headingPosition) - 180);

            autoTrajectories.drive.update();
            telemetry.update();
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
                module.clearBulkCache();
            }
        }
    }

    public void autoSequence(){
        if ((globalTimer > 28500) && goToPark && currentState != AutoState.IDLE){
            goToPark = false;
            currentState = AutoState.PARK;
            autoTrajectories.park(poseEstimate,2);
        }

        switch (currentState) {
            case DELAY:
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.UPRIGHT);
                outtakeSubsystem.pitchToInternalPID(PITCH_PURPLE_PIXEL_POSITION,1);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                outtakeSubsystem.miniTurretPointToBackdrop(correctedHeading);
                outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition);
                intakeSubsystem.intakeSlideInternalPID(0,1);
                if (delay(0)){
                    resetTimer();
                    currentState = AutoState.PRELOAD_DRIVE;
                    if (teamPropLocation == 1){
                      //  autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive1Front);
                        telemetry.addLine("left");
                    } else if (teamPropLocation == 2){
                      //  autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive2Front);
                        telemetry.addLine("center");
                    } else if (teamPropLocation == 3){
                      //  autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive3Front);
                        telemetry.addLine("right");
                    }
                }

                break;
            case PRELOAD_DRIVE:
                outtakeSubsystem.liftToInternalPID(3.9,1); // so rail doesn't hit

                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
                if (delay(1000)){
                    outtakeSubsystem.outtakeRailState(S != 1? OuttakeSubsystem.OuttakeRailState.RIGHT: OuttakeSubsystem.OuttakeRailState.LEFT);
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_PURPLE);
                    outtakeSubsystem.setOuttakePitchPurplePixelPosition();
                    // turret deposit logic
                    if (teamPropLocation == 1){
                        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.FRONT_PURPLE);
                    } else if (teamPropLocation == 2){
                        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                    } else if (teamPropLocation == 3){
                        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.BACK_PURPLE);
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
                    currentState = AutoState.PLACE_AND_INTAKE;
                    resetTimer();

                }
                break;

            case PLACE_AND_INTAKE:
                intakeSubsystem.intakeSpin(1);
                preExtendIntakeSlides();
                if (intakeSubsystem.intakeSlideTargetReached() || limitSwitches() || intakeSubsystem.pixelsInIntake()){ // limit switches don't touch in this case
                    if (delay(160)){ // ensure pixels are in robot
                        intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
                        autoTrajectories.outtakeDriveMiddlePath(poseEstimate,15, 30, -32);
                        resetTimer();
                        currentState = AutoState.TRANSFER_PIXEL;
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
                break;

            case AFTER_PURPLE_DRIVE:
                /* // this is the trigger statement for this case in case we need it later
                currentState = AutoState.AFTER_PURPLE_DRIVE;
                        if (teamPropLocation == 1){
                            autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.AfterPreloadDrive1Front);
                        } else if (teamPropLocation == 2){
                            autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.AfterPreloadDrive2Front);
                        } else if (teamPropLocation == 3){
                            autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.AfterPreloadDrive3Front);
                        }
                 */


                /*
                intakeSubsystem.intakeSlideInternalPID(0,1);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                intakeSubsystem.intakeSpin(-1);
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP);
                if (RED_AUTO? yPosition < -26.5*S:yPosition > -26.5*S){
                   // autoTrajectories.outtakeDriveMiddlePath(poseEstimate, 20);
                    if (teamPropLocation == 2){
                        autoTrajectories.outtakeDriveMiddlePath(poseEstimate,16, 30, -31);
                    } else if (teamPropLocation == 1){
                        autoTrajectories.outtakeDriveTurnEndPath(poseEstimate,16, -173,32, 2);
                    } else if (teamPropLocation == 3){
                        autoTrajectories.outtakeDriveTurnEndPath(poseEstimate,16, 173,32, 2);

                    }

                    resetTimer();
                    currentState = AutoState.TRANSFER_PIXEL;
                }
                 */
                break;

            case TRANSFER_PIXEL:
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                liftDown(0.1);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                intakeSubsystem.intakeSlideInternalPID(-20,1);
                if (numCycles == 0? delay(300): delay(220)){ // time for pixel holder to close/reverse
                    intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP);
                    // goes back into the stack
                    /*
                    if ((delay(450)) && timesIntoStack < 1 && numCycles != 0){ // go back into stack formore pixels case
                        if (!intakeSubsystem.pixelsInIntake()){
                            currentState = AutoState.GRAB_OFF_STACK;
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
                     */



                    intakeSubsystem.intakeSpin(0.8);
                    if ((intakeSubsystem.intakeSlidePosition < 6 || (intakeSubsystem.intakeSlidePosition < 8 && intakeSubsystem.chuteDetectorLimitSwitchValue))  && ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) < 0.1 ){
                        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.TRANSFER);
                        //outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                        telemetry.addLine("lift is ready to transfer");
                        currentState = AutoState.OUTTAKE_PIXEL;
                        intakeSubsystem.intakeSlideInternalPID(5,1); // power draw reasons
                        resetTimer();
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
                if (delay(4000)){
                    currentState = AutoState.PARK;
                    autoTrajectories.park(poseEstimate,2);
                }
                break;

            case OUTTAKE_PIXEL: // faster transfer
               // if (delay() 40 || (intakeSubsystem.chuteDetectorLimitSwitchValue && delay() 20)){
                if (numCycles != 0){
                    outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition);
                } else {
                    outtakeSubsystem.setOuttakePitchYellowPixelPosition();
                }

                if (numCycles !=0){
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE); // slides go out before arm so transfer is good
                } else {
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.YELLOW); // slides go out before arm so transfer is good
                }
                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                    if (delay(150)){ // time for grippers to close
                        if (numCycles == 0){ // for very first cycle
                            outtakeSubsystem.pitchToInternalPID(18,1);
                            //outtakeSubsystem.updateLiftTargetProfile(570);
                            outtakeSubsystem.liftTo(25,outtakeSubsystem.liftPosition,1);
                        } else if (numCycles == 1){
                            outtakeSubsystem.liftTo(30,outtakeSubsystem.liftPosition,1);
                            //outtakeSubsystem.updateLiftTargetProfile(650);
                            outtakeSubsystem.pitchToInternalPID(18,1);
                        } else if (numCycles == 2){
                            //outtakeSubsystem.updateLiftTargetProfile(734);
                            outtakeSubsystem.liftTo(26.5,outtakeSubsystem.liftPosition,1);
                            outtakeSubsystem.pitchToInternalPID(19,1);
                        } else if (numCycles == 3){
                            //outtakeSubsystem.updateLiftTargetProfile(780);
                            outtakeSubsystem.liftTo(30,outtakeSubsystem.liftPosition,1);
                            outtakeSubsystem.pitchToInternalPID(22,1);
                        } else if (numCycles == 4){
                            //outtakeSubsystem.updateLiftTargetProfile(780);
                            outtakeSubsystem.liftTo(30,outtakeSubsystem.liftPosition,1);
                            outtakeSubsystem.pitchToInternalPID(25,1);
                        }
                        //outtakeSubsystem.profileLiftCalculate();
                        if (numCycles != 2){
                            intakeSubsystem.intakeSlideInternalPID(330, 0.6);
                        } else {
                            intakeSubsystem.intakeSlideInternalPID(0, 1);
                        }

                        //ready's the chute after slides have gone out
                        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);

                        if (delay(100)){
                            railLogic();
                            if (delay(350)){ // once chute is down
                                outtakeSubsystem.miniTurretPointToBackdrop(correctedHeading);
                                pivotLogic();

                                if (!autoTrajectories.drive.isBusy() || (outtakeSubsystem.outtakeDistanceSensorValue < 6.9)) { // && numCycles != 0
                                    // line above determines when we drop the pixels
                                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                                    outtakeSubsystem.outtakeDistanceSensorValue = 100; // so that it doesn't do funky stuff
                                    numCycles += 1;
                                    timesIntoStack = 0;
                                    resetTimer();

                                    // just in case
                                    intakeSubsystem.intakeSpin(0);

                                    if (numCycles == 1){
                                        autoTrajectories.driveIntoStackStraight(poseEstimate,18,2,0); // might cause an issue here
                                        currentState = AutoState.DROP;
                                    } else if (numCycles == 2){
                                        autoTrajectories.driveIntoStackStraight(poseEstimate,18,2,0); // might cause an issue here
                                        currentState = AutoState.DROP;
                                    } else if (numCycles == 3){
                                        autoTrajectories.driveIntoStackStageFromMiddlePathStraightEndV2(poseEstimate,12, 30); // this is going to far lane
                                        currentState = AutoState.DROP;
                                    } else if (numCycles == 4){
                                        autoTrajectories.driveIntoStackStraight(poseEstimate,12,3,-3); // might cause an issue here
                                        currentState = AutoState.DROP;
                                    } else if (numCycles == 5){
                                        currentState = AutoState.PARK;
                                        autoTrajectories.park(poseEstimate,2);
                                    }
                                }
                            }
                        } else {
                        }
                    }
            //    }
                if (delay(400)){
                    intakeSubsystem.intakeSpin(0);
                } else {
                    intakeSubsystem.intakeSpin(-0.8);
                }
                break;
            case DROP:
                // waits for robot to move away far enough - minimize this
                if (delay(50)){
                    outtakeSubsystem.liftToInternalPID(-0.6,1);
                    outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
                    if (delay(110)){ // not knock pixels off the backdrop
                        //intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                        if (numCycles == 1){
                               intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.FOUR);
                        } else if (numCycles == 2){
                           intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE);
                        } else if (numCycles == 3){
                            // make intake arm height correct
                          intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP);
                        } else if (numCycles == 4) {
                         intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE);
                        }
                        currentState = AutoState.GRAB_OFF_STACK;
                        resetTimer();
                        intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);

                    }
                }
                break;

            // might need to make multiple of these states depending on the trajectory it follows -- again making methods would help alot with this
            case GRAB_OFF_STACK: // put each state in a method in antoher class and pass in timer variable

                outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);

                if ( numCycles > 2){ // wait for the outtake rail to retract
                    if (delay(400)){
                        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                    }
                } else {
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                    outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                }

                outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS, 1);
                if (delay(100)){
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
                }

                liftDown(0.08);


                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);

                if (xPosition < 14 && Math.abs(correctedHeading) < 2){ // the robot is driving from
                    intakeSubsystem.intakeSlideInternalPID(INTAKE_SLIDE_AUTO_LONG_PRESET,1);
                    if (xPosition < 12) { // for some reason we can't extend the slides unless we are further in
                        intakeSubsystem.intakeSpin(1);
                        if (!autoTrajectories.drive.isBusy() || intakeSubsystem.leftArmLimitSwitchValue || intakeSubsystem.rightArmLimitSwitchValue
                            || (xPosition < -24 && intakeSubsystem.pixelsInIntake())){ // do stuff with sensor to make better
                            if (numCycles > 2) {
                                autoTrajectories.outtakeDriveFromStraightTurnEndStageV2(poseEstimate,16, 175, 4);
                            } else {
                                autoTrajectories.outtakeDriveMiddlePath(poseEstimate,16, 26, MiddleLaneYDeposit);
                            }
                            currentState = AutoState.AFTER_GRAB_OFF_STACK;
                            resetTimer();

                        }
                    }
                } else if (numCycles != 3){
                    intakeSubsystem.intakeSlideInternalPID(INTAKE_SLIDE_AUTO_LONG_PRESET,1);
                }
                /*
                if (timesIntoStack !=0){
                    if (delay() 150){
                        intakeSubsystem.intakeSpin(-1);
                    } else {
                        intakeSubsystem.intakeSpin(1);
                    }
                }
                 */
                if (intakeSubsystem.pixelsInIntake() && timesIntoStack !=0){ //this isn't true unless timesIntoStack has happened at least once
                    currentState = AutoState.AFTER_GRAB_OFF_STACK;
                    resetTimer();
                    if (numCycles > 2) {
                        autoTrajectories.outtakeDriveFromStraightTurnEndStageV2(poseEstimate,16, 169, 4);
                    } else {
                        if (numCycles == 2){
                            autoTrajectories.outtakeDriveMiddlePath(poseEstimate,10, 26, MiddleLaneYDeposit);
                        } else {
                            autoTrajectories.outtakeDriveMiddlePath(poseEstimate,13, 26, MiddleLaneYDeposit);
                        }
                    }
                }
                break;
            case AFTER_GRAB_OFF_STACK:
                /*
                if (numCycles == 1 || numCycles == 3) {
                    //intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.FOUR);
                }
                 */
                if (numCycles == 2){
                    intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                }
                //TODO abstract all of the intakearm logic to its own method
                if (numCycles == 2 || numCycles >= 4 ? delay(170) : delay(120)){
                    resetTimer(); // resets timer
                    currentState = AutoState.TRANSFER_PIXEL;
                    intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
                }
                break;
            case PARK:
                if (delay(200)){
                   currentState = AutoState.IDLE; // doesn't have to drive anywhere, already in position hopefully
                    resetTimer(); // resets timer
                }
                break;

            case IDLE:
                intakeSubsystem.intakeSlideInternalPID(-6,1);
                outtakeSubsystem.liftToInternalPID(-8,1);
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
                outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS, 1);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                if (delay(400)){
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                }
                outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
                break;
        }
    }

    public void pivotLogic (){
        if (numCycles == 0){
            if (teamPropLocation == 1){
                if (BLUE_AUTO){
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
                }
                else {
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
                }
            } else if (teamPropLocation == 2){
                if (RED_AUTO){
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_RIGHT_FLIPPED);
                }
                else {
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_LEFT_FLIPPED);
                }
            } else if (teamPropLocation == 3){
                if (RED_AUTO){
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
                }
                else {
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
                }
            }
        } else if (numCycles < 3){
            if (BLUE_AUTO){
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
            }
            else {
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
            }
        } else {
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
        }

    }
    public void railLogic(){
        if (numCycles == 0){
            if (teamPropLocation == 2){
                outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER_YELLOW);
            } else if (teamPropLocation == 1){
                outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.LEFT);
            } else if (teamPropLocation == 3){
                outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.RIGHT);
            }
        } else if (numCycles > 2){
            outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.RIGHT);
        } else {
            outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
        }
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
            intakeSubsystem.intakeSlideInternalPID(275,0.9);
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
}


