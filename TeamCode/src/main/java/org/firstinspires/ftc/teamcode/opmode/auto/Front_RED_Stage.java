package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.*;

@Autonomous(name = "Front Stage 2+9", group = "Autonomous")
public class Front_RED_Stage extends LinearOpMode {

    int numCycleForDifferentLane = 0;

    //Accessories
    AutoSequences auto = new AutoSequences(telemetry);
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
        PARK,
        GO_BACK_FOR_YELLOW,
        IDLE,
    }

    AutoState currentState;

    @Override
    public void runOpMode() throws InterruptedException {

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
            if (teamPropLocation == 1){
                telemetry.addLine("Left");
            } else if (teamPropLocation == 2){
                telemetry.addLine("Center");
            } else if (teamPropLocation == 3){
                telemetry.addLine("Right");
            }
            telemetry.addData("S", S);
            telemetry.update();
        }


        waitForStart();
        if (isStopRequested()) return;
        // runs instantly once
        auto.afterWaitForStart(auto.autoTrajectories.startPoseFront, false);
        currentState = AutoState.DELAY;


        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
                module.clearBulkCache();
            }

            auto.mainAutoLoop(
                    currentState == AutoState.OUTTAKE_PIXEL && xPosition > 20,
                    currentState == AutoState.GRAB_OFF_STACK || currentState == AutoState.AFTER_GRAB_OFF_STACK || currentState == AutoState.PLACE_AND_INTAKE,
                    currentState != AutoState.PRELOAD_DRIVE && currentState != AutoState.OUTTAKE_PIXEL);


            if (place == Place.LEFT){
                RAIL_CENTER_YELLOW_STAGE_POS += inchesToTicksRailCorrected(-3);
            }

            autoSequence();
            loopTime.delta();
            telemetry.addData("LoopTime", loopTime.getDt() / 1_000_000);
            //telemetry.addData("Hz", loopTime.getHz());
            telemetry.addData("Auto State", currentState);
            telemetry.addData("intakeSlidePosition", auto.intakeSubsystem.intakeSlidePosition);
            telemetry.addData("Case", place);
            telemetry.addData("Rail target", auto.cameraHardware.getRailTarget(auto.correctedHeading,ticksToInchesSlidesMotor(auto.outtakeSubsystem.liftPosition), auto.outtakeSubsystem.pitchEncoderPosition));
            telemetry.addData("X pos", xPosition);
            telemetry.addData("TagTarget", auto.cameraHardware.getTargetTag());
            telemetry.addData("Tag we see", auto.cameraHardware.getSeenTags());
            telemetry.update();

        }
        auto.storePoseEndAuto(poseEstimate);
    }

    public void autoSequence(){
        auto.goToPark(currentState == AutoState.IDLE,2);
        switch (currentState) {
            case DELAY:
                if(auto.delayState(0)){
                    if (teamPropLocation == 1){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive1FrontStage);
                        telemetry.addLine("left");
                    } else if (teamPropLocation == 2){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive2FrontStage);
                        telemetry.addLine("center");
                    } else if (teamPropLocation == 3){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive3FrontStage);
                        telemetry.addLine("right");
                    }
                    currentState = AutoState.PRELOAD_DRIVE;
                }
                break;
            case PRELOAD_DRIVE:
                double railDelay = 30000;
                if (yPosition > -20){
                    railDelay = 0;
                }
                if(auto.preloadDriveState(false, true,railDelay, 0.4)){
                    currentState = AutoState.PLACE_AND_INTAKE;
                }
                break;

            case PLACE_AND_INTAKE:
                if (auto.placeAndIntakeFrontMIDTRUSS(230,0.4, true)){
                    if (auto.goBackForYellowPixel){
                        currentState = AutoState.GO_BACK_FOR_YELLOW;
                    } else {
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
                break;

            case GO_BACK_FOR_YELLOW:
                if (auto.reExtendSLidesForYellow(1500,0.34)){
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
                break;

            case TRANSFER_PIXEL:
                auto.goBackToStack(3,10,-29);
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
                int intakeSlideTarget = 330; // pre-extend intake for most cycles
                Trajectory intakeTrajectory = null;
                boolean extendStraightAway = true;
                boolean openGrippers = true;
                if (numCycles == 0){
                    intakeSlideTarget = 10;
                    if (xPosition > 8){
                        auto.outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                    }
                    if (xPosition > 17){
                        // stop yellow detection
                        auto.cameraHardware.pausePreloadProcessor();
                        railTarget = auto.cameraHardware.getRailTarget(auto.correctedHeading, ticksToInchesSlidesMotor(auto.outtakeSubsystem.liftPosition), auto.outtakeSubsystem.pitchEncoderPosition);
                        railLogic.setRailTargetFromAprilTag(railTarget);
                    }

                    pitchTarget = 26; // yellow pixel should be pitched higher
                    liftTarget = 13;
                    extendStraightAway = false;
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
                    intakeSlideTarget = 50;
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
                    if (numCycles == 1){
                        intakeTrajectory = auto.autoTrajectories.driveBackToDropYellow(poseEstimate,10,5);
                    }
                    if (numCycles == 2){
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackStraightTrajectory(poseEstimate,20,3,0,-27.5, -20);
                    } else if (numCycles == 3){ // turning into the stacks
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectoryStage(poseEstimate,20,-3,-160,3,0);
                    } else if (numCycles == 4){
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectoryStage(poseEstimate,20,-3,-160,3,0);
                    }

                    auto.resetPosWithAprilTags(3);

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
                    if (auto.delay(630)){
                        auto.outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                    }
                    delayTime = 800;
                    armHeight = 4;
                } else if (numCycles == 2){
                    armHeight = 3;
                } else if (numCycles == 3){
                    armHeight = 5;
                } else if (numCycles == 4) {
                    armHeight = 3;
                }

                if (auto.drop(armHeight, false, delayTime)){
                    if (numCycles == 1){ // for very first cycle
                        intakeTrajectoryAfterDrop = auto.autoTrajectories.driveIntoStackStraightTrajectory(poseEstimate,20,3,0,-27,-17);
                        if (intakeTrajectoryAfterDrop != null){
                            auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectoryAfterDrop);
                        }
                    }
                    if (auto.globalTimer > 25500){
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
                boolean extendSlides = true;
                if (numCycles == 1){
                    delayBeforeRetracting = 800;
                }
                if (numCycles == 3){
                    intakeSlidePosition = 760;
                    extendSlides = false;
                } else if (numCycles == 4){
                    intakeSlidePosition = 800;
                    extendSlides = false;
                }
                if (auto.grabOffStack(numCycleForDifferentLane, true, extendSlides,3, intakeSlidePosition, delayBeforeRetracting)){
                    currentState = AutoState.AFTER_GRAB_OFF_STACK;
                    Trajectory outtakeTrajectory = null;
                    if (numCycles < 4) {
                        // this is the old spline path
                         outtakeTrajectory = auto.autoTrajectories.simplifiedOuttakeDrive(poseEstimate,17, 175, 4);
                        // outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,18, 175, 4);
                    }
                    if (outtakeTrajectory != null){
                        auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);
                    }
                }
                break;
            case AFTER_GRAB_OFF_STACK:
                if (auto.afterGrabOffStack(2,3, 250,140)){
                    currentState = AutoState.TRANSFER_PIXEL;
                    if (numCycles >= 3) { // for the longer delay we follow the trajectory after the wait - just so its more consistent hopefully
                        Trajectory outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,14, 175, 4);
                        auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);
                    }
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


