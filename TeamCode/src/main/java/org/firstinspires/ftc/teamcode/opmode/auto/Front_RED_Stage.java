package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.*;

@Autonomous(name = "Front Red Stage Auto", group = "Autonomous")
public class Front_RED_Stage extends LinearOpMode {

    int numCycleForDifferentLane = 0;
    double delayForYellow = 0; // this is in seconds
    boolean frontOrBackAuto;

    //Accessories
    AutoSequences auto = new AutoSequences(telemetry,3);
    LoopTime loopTime = new LoopTime();
    // this works because the parameters being passed in don't change throughout the opmode
    // same cycle numbers
    AutoRail railLogic = new AutoRail(numCycleForDifferentLane,0, auto,true,3);
    AutoPivot pivotLogic = new AutoPivot(numCycleForDifferentLane,0, auto, telemetry,3);
    private double railTarget;


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
        GO_BACK_FOR_YELLOW,
        GO_BACK_FOR_WHITES,
        PARK,
        IDLE,
        DELAY_BACK,
        PRELOAD_DRIVE_BACK,
        PLACE_AND_INTAKE_BACK,
        PRELOAD_DRIVE_CASE_3,
        AFTER_PRELOAD_DRIVE_3
    }

    AutoState currentState;

    @Override
    public void runOpMode() throws InterruptedException {

        frontOrBackAuto = true;
        SetAuto.setRedAuto();
        auto.setGamepad1(gamepad1);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            module.clearBulkCache();
        } //

        auto.initAutoHardware(hardwareMap,this);

        // trajectories that aren't changing should all be here
        while (!isStarted()) { // initialization loop
            auto.intializationLoop(frontOrBackAuto);
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
        auto.afterWaitForStart(!frontOrBackAuto, frontOrBackAuto? auto.autoTrajectories.startPoseFront: auto.autoTrajectories.startPoseBack);
        if (frontOrBackAuto){
            currentState = AutoState.DELAY;
        } else {
            currentState = AutoState.DELAY_BACK;
        }

        // can set drive constraints here
        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
                module.clearBulkCache();
            }

            auto.mainAutoLoop(
                    currentState == AutoState.OUTTAKE_PIXEL && xPosition > 20,
                    currentState == AutoState.GRAB_OFF_STACK || currentState == AutoState.AFTER_GRAB_OFF_STACK || currentState == AutoState.PLACE_AND_INTAKE,
                    currentState != AutoState.PRELOAD_DRIVE && currentState != AutoState.OUTTAKE_PIXEL);

            autoSequence();
            loopTime.delta();
            telemetry.addData("numCycles", numCycles);
            telemetry.addData("Preload", auto.cameraHardware.getPreloadYellowPose());
            telemetry.addData("LoopTime", loopTime.getDt() / 1_000_000);
            //telemetry.addData("Hz", loopTime.getHz());
            telemetry.addData("Auto State", currentState);
            telemetry.addData("intakeSlidePosition", auto.intakeSubsystem.intakeSlidePosition);
            telemetry.addData("intakeSlidePosition", auto.intakeSubsystem.intakeSlidePosition);

            telemetry.update();

        }
        auto.storePoseEndAuto(poseEstimate);
    }

    public void autoSequence(){

        if (auto.goToPark(currentState == AutoState.IDLE,2)){
            currentState = AutoState.IDLE;
        }

        switch (currentState) {
            case DELAY_BACK:
                if(auto.delayState(0)){
                    currentState = AutoState.PRELOAD_DRIVE_BACK;
                    if (teamPropLocation == 1){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive1);
                    } else if (teamPropLocation == 2){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive2);
                    } else if (teamPropLocation == 3){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive3);
                    }
                }
                break;
            case PRELOAD_DRIVE_BACK:
                if(auto.preloadDriveStateBack()){
                    currentState = AutoState.PLACE_AND_INTAKE_BACK;
                }
                break;
            case PLACE_AND_INTAKE_BACK:
                if (auto.placeAndIntakeBackSTage(0.8)){
                    currentState = AutoState.DROP;
                }
                break;

            case DELAY:
                if(auto.delayState(0)){
                    if (teamPropLocation == 1){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive1FrontStage);
                        telemetry.addLine("left");
                    } else if (teamPropLocation == 2){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive2FrontStage);
                        telemetry.addLine("center");
                    } else if (teamPropLocation == 3){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive3FrontFirst);
                        telemetry.addLine("right");
                    }
                    if (teamPropLocation != 3){
                        currentState = AutoState.PRELOAD_DRIVE;
                    } else {
                        currentState = AutoState.PRELOAD_DRIVE_CASE_3;
                    }

                }
                break;

            case PRELOAD_DRIVE_CASE_3:
                if (auto.preloadDriveState3()){
                    currentState = AutoState.AFTER_PRELOAD_DRIVE_3;
                    auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive3FrontSecond);
                }
                break;

            case AFTER_PRELOAD_DRIVE_3:
                if (auto.afterPreloadDriveState3()){
                    currentState = AutoState.PLACE_AND_INTAKE;
                    auto.frontThirdCase = true;
                }
                break;

            case PRELOAD_DRIVE:
                if(auto.preloadDriveState(false, true,950, .5, false)){
                    currentState = AutoState.PLACE_AND_INTAKE;
                }
                break;

            case PLACE_AND_INTAKE:
                if (auto.placeAndIntakeFrontMIDTRUSS(250,.5, true)){
                    if (auto.goBackForYellowPixel){
                        currentState = AutoState.GO_BACK_FOR_YELLOW;
                    } else {
                        if (auto.GlobalTimer.seconds() > delayForYellow){
                            Trajectory startDrive = null;
                            if (teamPropLocation == 2){
                                startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple2;
                            } else if (teamPropLocation == 1){
                                startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple1;
                            } else if (teamPropLocation == 3){
                                startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple3;
                            }
                            auto.autoTrajectories.drive.followTrajectoryAsync(startDrive);
                            currentState = AutoState.TRANSFER_PIXEL;
                        }
                    }
                }
                break;

            case GO_BACK_FOR_YELLOW:
                if (auto.reExtendSLidesForYellow(1200,0.35)){
                    if (auto.GlobalTimer.seconds() > delayForYellow){
                        Trajectory startDrive = null;
                        if (teamPropLocation == 2){
                            startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple2;
                        } else if (teamPropLocation == 1){
                            startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple1;
                        } else if (teamPropLocation == 3){
                            startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple3;
                        }
                        auto.autoTrajectories.drive.followTrajectoryAsync(startDrive);
                        auto.resetTimer();
                        currentState = AutoState.TRANSFER_PIXEL;
                    }
                }
                break;

            case TRANSFER_PIXEL:
                if (numCycles < 3){
                    auto.goBackToStack(3,6,-29.3);
                }
                if (auto.goBackToStack){
                    currentState = AutoState.GRAB_OFF_STACK;
                }
                if (auto.transferPixel()){
                    currentState = AutoState.OUTTAKE_PIXEL;
                }
                break;

            case OUTTAKE_PIXEL:
                //outtaking lengths for each cycle
                double liftTarget = 0; // could cause issues if these stay zero
                int pitchTarget = 0;
                int intakeSlideTarget = 350; // pre-extend intake for most cycles
                Trajectory intakeTrajectory = null;
                boolean openGrippers = true;
                boolean extendStraightAway = false;
                if (xPosition > -10 && numCycles != 0){ // custom extend on the first cycle
                    auto.autoTrajectories.extendSlidesAroundTruss = true;
                }
                if (numCycles == 0){
                    intakeSlideTarget = 10;
                    if (xPosition > 8){
                        auto.outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                    }
                    if (teamPropLocation != 1? xPosition > 21: teamPropLocation == 2? xPosition > 19: xPosition > 17){
                        // stop yellow detection
                        auto.cameraHardware.pausePreloadProcessor();
                        railTarget = auto.cameraHardware.getRailTarget(auto.correctedHeading, ticksToInchesSlidesMotor(auto.outtakeSubsystem.liftPosition), auto.outtakeSubsystem.pitchEncoderPosition);
                        railLogic.setRailTargetFromAprilTag(railTarget);
                    }

                    pitchTarget = 26; // yellow pixel should be pitched higher
                    liftTarget = 13;
                    openGrippers = false;
                } else if (numCycles == 1)
                {
                    pitchTarget = 23;
                    liftTarget = 30;
                } else if (numCycles == 2){
                    pitchTarget = 24;
                    liftTarget = 30.5;
                } else if (numCycles == 3){
                    pitchTarget = 26;
                    liftTarget = 31;
                    intakeSlideTarget = 0;
                } else if (numCycles == 4){
                    pitchTarget = 26;
                    liftTarget = 31;
                    intakeSlideTarget = 50;
                    auto.goToParkAfterOuttaking = true;
                }
                boolean outtakePixelFinished = auto.outtakePixel(auto.correctedHeading,liftTarget,pitchTarget,intakeSlideTarget,railLogic,pivotLogic,extendStraightAway, false, false, openGrippers);
                if (auto.goToParkAfterOuttaking && outtakePixelFinished || (teamPropLocation == 1 && numCycles == 3)){ // if team prop location is 1 we don't want more pixels
                    intakeTrajectory = auto.autoTrajectories.parkTrajectory(poseEstimate,2);
                    currentState = AutoState.PARK;
                    auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectory);
                }
                else if (outtakePixelFinished){
                    auto.resetPosWithAprilTags(3);

                    if (numCycles == 1){
                        intakeTrajectory = auto.autoTrajectories.driveBackToDropYellow(poseEstimate,10,5);
                    }
                    if (numCycles == 2){
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackStraightTrajectory(poseEstimate,20,3,0,-27.5, -20);
                    } else if (numCycles == 3){ // turning into the stacks
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectoryStage(poseEstimate,18,-2.6,-174,3,0);
                    } else if (numCycles == 4){
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectoryStage(poseEstimate,20,-0.5,-174,3,0);
                    }

                    if (intakeTrajectory != null){
                        auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectory);
                    }
                    currentState = AutoState.DROP;
                }
                break;
            case DROP:
                Trajectory intakeTrajectoryAfterDrop;
                double delayTime = 90;
                int armHeight = 0;
                if (numCycles == 1){
                    if (auto.delay(500)){
                        auto.outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                    }
                    delayTime = frontOrBackAuto? 690:350;
                    armHeight = 5;
                } else if (numCycles == 2){
                    armHeight = 3;
                } else if (numCycles == 3){
                    armHeight = 5;
                } else if (numCycles == 4) {
                    armHeight = 3;
                }

                if (auto.drop(armHeight, false, delayTime)){
                    if (numCycles == 1){ // for very first cycle
                        if (frontOrBackAuto){
                            if (teamPropLocation == 1){
                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterYellowStage1);
                            } else if (teamPropLocation == 2){
                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterYellowStage2);
                            } else if (teamPropLocation == 3){
                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterYellowStage3);
                            }
                        } else { // for the back side autos we just run this straight away
                            intakeTrajectoryAfterDrop = auto.autoTrajectories.driveIntoStackStraightTrajectory(poseEstimate,20,3,0,-27,-17);
                            if (intakeTrajectoryAfterDrop != null){
                                auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectoryAfterDrop);
                            }
                        }
                    }
                    if (auto.GlobalTimer.seconds() > 25.5){
                        auto.parkIfStuck = true; // should force into park before doing another cycle
                    } else {
                        currentState = AutoState.GRAB_OFF_STACK;
                    }
                }
                break;
            case GRAB_OFF_STACK:
                if (xPosition < -17.7){
                    auto.autoTrajectories.extendSlidesAroundStage = true;
                }
                double delayBeforeRetracting = 0;
                int intakeSlidePosition = INTAKE_SLIDE_AUTO_LONG_PRESET;
                boolean extendSlides = false;
                double xPosSlideThresh = -10;
                if (numCycles == 1){
                    delayBeforeRetracting = 500;
                    if (S == 1? xPosition < 10 : xPosition < -17){
                        auto.autoTrajectories.extendSlidesAroundStage = true;
                    }
                   /* if (S == 1){
                        extendSlides = true;
                    }*/
                }
                if (numCycles == 2){
                    if (S == 1? xPosition < 10 : xPosition < -14){
                        auto.autoTrajectories.extendSlidesAroundStage = true;
                    }/*if (S == 1){
                        extendSlides = true;
                    }*/
                }
                if (numCycles == 3){
                    intakeSlidePosition = 800;
                    extendSlides = false;
                } else if (numCycles == 4){
                    intakeSlidePosition = 800;
                    extendSlides = false;
                }
                if (auto.grabOffStack(numCycleForDifferentLane, true, extendSlides,3, intakeSlidePosition, delayBeforeRetracting, xPosSlideThresh)){
                    currentState = AutoState.AFTER_GRAB_OFF_STACK;
                    Trajectory outtakeTrajectory = null;
                    if (numCycles >= 3) { // for the longer delay we follow the trajectory after the wait - just so its more consistent hopefully
                        outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromAngleTurnEndTrajectory(poseEstimate, 14, 32, -5, 4, 3,0);
                        //outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,14, 175, 4);
                        auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);
                    }
                    else{// (numCycles < 4) {
                        // this is the old spline path
                        outtakeTrajectory = auto.autoTrajectories.simplifiedOuttakeDrive(poseEstimate,16, 177, -6,3);
                        //      outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,18, 175, 4);
                        //     outtakeTrajectory = auto.autoTrajectories.outtakeDriveMiddlePathTrajectory(poseEstimate,18, 175, 4);
                    }
                    if (outtakeTrajectory != null){
                        auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);
                    }
                }
                break;
            case AFTER_GRAB_OFF_STACK:
                if (auto.afterGrabOffStack(2,3, 250,140)){
                    currentState = AutoState.TRANSFER_PIXEL;

                }
                break;
            case PARK:
                if (auto.park()){
                    currentState = AutoState.IDLE;
                }
                break;
            case IDLE:
                auto.idle();
                break;
        }

    }
}