package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.*;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

@Autonomous(name = "Front Middle 2+11", group = "Autonomous")
public class Front_RED_MIDDLE_CYCLE extends LinearOpMode {

    int numCycleForDifferentLane = 2;

    //Accessories
    AutoSequences auto = new AutoSequences(telemetry,2);
    LoopTime loopTime = new LoopTime();
    // this works because the parameters being passed in don't change throughout the opmode
    // same cycle numbers
    AutoRail railLogic = new AutoRail(numCycleForDifferentLane,0, auto,true,2);
    AutoPivot pivotLogic = new AutoPivot(numCycleForDifferentLane,0, auto, telemetry,2);


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
            auto.intializationLoop(true);
            telemetry.addData("inited middle autos 1", auto.autoTrajectories.initedMiddle211trajecs);
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
        auto.afterWaitForStart(false, auto.autoTrajectories.startPoseFront);
        currentState = AutoState.DELAY;

        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
                module.clearBulkCache();
            }

            auto.mainAutoLoop(
   currentState == AutoState.OUTTAKE_PIXEL && xPosition > 20,
(currentState == AutoState.GRAB_OFF_STACK && xPosition < -25) || currentState == AutoState.AFTER_GRAB_OFF_STACK || currentState == AutoState.PLACE_AND_INTAKE,
currentState != AutoState.PRELOAD_DRIVE && currentState != AutoState.OUTTAKE_PIXEL);

            autoSequence();
            loopTime.delta();
            telemetry.addData("numcycle", numCycles);
            //telemetry.addData("goToParkAfterOuttaking", auto.goToParkAfterOuttaking);
            telemetry.addData("LoopTime", loopTime.getDt() / 1_000_000);
           // telemetry.addData("Hz", loopTime.getHz());
            telemetry.addData("Auto State", currentState);
            telemetry.addData("intakeSlidePosition", auto.intakeSubsystem.intakeSlidePosition);
            telemetry.update();

        }
        auto.storePoseEndAuto(poseEstimate);
    }

    public void autoSequence(){
        auto.goToPark(currentState == AutoState.IDLE,2);
        switch (currentState) {
            case DELAY:
                if (auto.delayState(0)){
                    currentState = AutoState.PRELOAD_DRIVE;
                    if (teamPropLocation == 1){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive1Front);
                        telemetry.addLine("left");
                    } else if (teamPropLocation == 2){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive2Front);
                        telemetry.addLine("center");
                    } else if (teamPropLocation == 3){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive3Front);
                        telemetry.addLine("right");
                    }
                }

                break;
            case PRELOAD_DRIVE:
                if(auto.preloadDriveState(true,true,1000,0.7, true)){
                    currentState = AutoState.PLACE_AND_INTAKE;
                }

                break;

            case PLACE_AND_INTAKE:
                if (auto.placeAndIntakeFrontMIDTRUSS(170,1, true)){
//                    Trajectory startDrive = auto.autoTrajectories.outtakeDriveMiddlePathTrajectory(poseEstimate,15, 29, -32);
//                    auto.autoTrajectories.drive.followTrajectoryAsync(startDrive);
                    auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.firstOuttakeMiddleDrive);
                    currentState = AutoState.TRANSFER_PIXEL;
                }
                break;

            case TRANSFER_PIXEL:
                if (auto.goBackToStack){
                    currentState = AutoState.GRAB_OFF_STACK;
                }
                if (numCycles < 3){
                    auto.goBackToStack(2,6,-31,180);
                } else {
                    auto.goBackToStack(3,6,-31,180);
                }
                if (auto.transferPixel(numCycles==0?false:true)){
                    currentState = AutoState.OUTTAKE_PIXEL;
                }

                break;

            case OUTTAKE_PIXEL:
                //outtaking lengths for each cycle

                double liftTarget = 0; // could cause issues if these stay zero
                int pitchTarget = 0;
                int intakeSlideTarget = 330; // pre-extend intake for most cycles
                boolean extendSlidesStraightAway = true;
                Trajectory intakeTrajectory = null;
                if (numCycles == 0){ // for very first cycle
                    pitchTarget = 21;
                    liftTarget = 24;
                } else if (numCycles == 1){
                    pitchTarget = 21;
                    liftTarget = 27;
                } else if (numCycles == 2){
                    intakeSlideTarget = 0;
                    pitchTarget = 24;
                    liftTarget = 27;
                } else if (numCycles == 3){
                    pitchTarget = 22;
                    liftTarget = 27.5;
                } else if (numCycles == 4){
                    pitchTarget = 26;
                    liftTarget = 27.5;
                    auto.goToParkAfterOuttaking = true;
                } else if (numCycles == 5){
                    pitchTarget = 26;
                    liftTarget = 27.5;
                    auto.goToParkAfterOuttaking = true;
                } else if (numCycles == 6){
                    //auto.goToParkAfterOuttaking = true;
                }
                boolean outtakePixelFinished = auto.outtakePixel(auto.correctedHeading,liftTarget,pitchTarget,intakeSlideTarget,railLogic,pivotLogic,extendSlidesStraightAway, true, false, true, true);
                if (auto.goToParkAfterOuttaking && outtakePixelFinished){
                    intakeTrajectory = auto.autoTrajectories.parkTrajectory(poseEstimate,2);
                    auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectory);
                    currentState = AutoState.PARK;
                }
                else if (outtakePixelFinished){
                    if (numCycles == 1){ // for very first cycle
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackStraightMiddleTrajectory(poseEstimate,18,2,-0.7,-29.5, -24.7,180);
                    } else if (numCycles == 2){
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackStraightMiddleTrajectory(poseEstimate,18,2,-1,-29.5, -24.7, 180);
                    } else if (numCycles == 3){
                        if (teamPropLocation == 1){
                            intakeTrajectory = auto.autoTrajectories.driveIntoStackStageFromMiddlePathStraightEndV2; //doesn't generate on the fly
                        } else {
                            intakeTrajectory = auto.autoTrajectories.driveIntoStackStageFromMiddlePathStraightEndV1; //doesn't generate on the fly
                        }
                    } else if (numCycles == 4){
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackStraightTrajectory(poseEstimate,20,3,-3,-28.5, -25,180);
                    } else if (numCycles == 5){
                      //  intakeTrajectory = auto.autoTrajectories.driveIntoStackStraightTrajectory(poseEstimate,20,3,-3);

                   //     intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectory(poseEstimate,18,-3,-160,3,0);
                    }
                    // happens when the robot is stopped - after generating trajectories so we get a good reading
                    auto.resetPosWithAprilTags(2,S == -1? false:true);

                    if (intakeTrajectory != null){
                        auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectory);
                    }
                    currentState = AutoState.DROP;
                }
                break;
            case DROP:
                double delayTime = 110;
                boolean retractLift = true;
                int armHeight = 0;
                if (numCycles == 1){
                    armHeight = 4;
                    delayTime = 200;
                    retractLift = false;
                } else if (numCycles == 2){
                    armHeight = 1;
                } else if (numCycles == 3){
                    armHeight = 6;
                } else if (numCycles == 4) {
                    armHeight = 3;
                }

                if (auto.drop(armHeight, retractLift,delayTime)){
                currentState = AutoState.GRAB_OFF_STACK;
                }
                break;
            case GRAB_OFF_STACK:


                double delayBeforeRetracting = 200;
                int intakeSlidePosition = INTAKE_SLIDE_AUTO_LONG_PRESET;
                boolean extendSlides = false;
                double xPosSlideThresh = -10;
                boolean retractSlides = false;
                double xSplineValue = 6;
                double yOffset = 4;
                double endTangent = -6;
                double slideSpeed = 1;
                double xEnd = 29.2;

                if (numCycles == 2 || numCycles == 1){
                    extendSlides = true;
                }
                if (numCycles == 3){
                    xEnd = 29;
                    delayBeforeRetracting = 650;
                    retractSlides = true;
                    slideSpeed = 0.8;
                    if (xPosition < 14){//xPosition < -17
                        auto.autoTrajectories.extendSlidesAroundStage = true;
                    }
                    /*yOffset = 4;
                    xSplineValue = 6;*/
                   /* if (S == 1){
                        extendSlides = true;
                    }*/
                }
                if (numCycles == 4){
                    slideSpeed = 0.8;
                    xPosSlideThresh = -12;
                    xEnd = 29.6;
                    if (xPosition < 15){ //  xPosition < -14
                        auto.autoTrajectories.extendSlidesAroundStage = true;
                    }/*if (S == 1){
                        extendSlides = true;
                    }*/
                }

                if (numCycles == 5){
                    //slideSpeed = 1;
                    xEnd = 30.8;
                    intakeSlidePosition = 838;
                    xSplineValue = 3;
                    yOffset = 4.3;
                    extendSlides = false;
                    retractSlides = true;
                    endTangent = -7.3;
                } else if (numCycles == 6){
                    xEnd = 32.3;
                    intakeSlidePosition = 850;
                    xSplineValue = 3;
                    yOffset = 4.5;
                    extendSlides = false;
                    retractSlides = true;
                    endTangent = -7.3;
                }
                 if (auto.grabOffStack(numCycleForDifferentLane, true, extendSlides ,5, intakeSlidePosition, delayBeforeRetracting, xPosSlideThresh, retractSlides,slideSpeed)){
                    currentState = AutoState.AFTER_GRAB_OFF_STACK;
                    Trajectory outtakeTrajectory;
                    if (numCycles > 4) {
                        outtakeTrajectory = auto.autoTrajectories.simplifiedOuttakeDrive(poseEstimate, numCycles >= 3 ? 20 : 21, 176.8, endTangent, yOffset, xSplineValue, xEnd);

                        outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate, 17, 175, 4,15);
                    }
                    else if (numCycles > 2) {
                        // this seems to be the best trajectory to follow
                        outtakeTrajectory = auto.autoTrajectories.simplifiedOuttakeDrive(poseEstimate,17, 175, -5,4, 14, 29.2);
                    } else {
                        outtakeTrajectory = auto.autoTrajectories.outtakeDriveMiddlePathTrajectory(poseEstimate,15, numCycles == 1? 26: 26.3, MiddleLaneYDeposit, 20);
                    }
                    auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);

                }
                break;
            case AFTER_GRAB_OFF_STACK:
                 if (auto.afterGrabOffStack(2,3, 120,90)){
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


