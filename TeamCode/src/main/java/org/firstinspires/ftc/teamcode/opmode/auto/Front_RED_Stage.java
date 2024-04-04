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



        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            module.clearBulkCache();
        } //

        auto.initAutoHardware(hardwareMap,this);



        // trajectories that aren't changing should all be here
        while (!isStarted()) { // initialization loop
            auto.intializationLoop();
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
        auto.afterWaitForStart();
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


            autoSequence();
            loopTime.delta();
            telemetry.addData("LoopTime", loopTime.getDt() / 1_000_000);
            //telemetry.addData("Hz", loopTime.getHz());
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
                if(auto.preloadDriveState(false, true,1600, 0.4)){
                    currentState = AutoState.PLACE_AND_INTAKE;
                }
                break;


            case PLACE_AND_INTAKE:
                if (auto.placeAndIntakeFrontMIDTRUSS(230,0.4)){
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
                if (numCycles == 0){
                    intakeSlideTarget = 10;
                    if (auto.delay(100)){
                        auto.outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                    }
                    pitchTarget = 20; // yellow pixel should be pitched higher
                    liftTarget = 19;
                    extendStraightAway = false;
                } else if (numCycles == 1){
                    pitchTarget = 20;
                    liftTarget = 30;
                } else if (numCycles == 2){
                    pitchTarget = 24;
                    liftTarget = 30.5;
                    intakeSlideTarget = 50;
                } else if (numCycles == 3){
                    pitchTarget = 26;
                    liftTarget = 31;
                    intakeSlideTarget = 50;
                } else if (numCycles == 4){
                    pitchTarget = 26;
                    liftTarget = 31;
                    auto.goToParkAfterOuttaking = true;
                }
                boolean outtakePixelFinished = auto.outtakePixel(auto.correctedHeading,liftTarget,pitchTarget,intakeSlideTarget,railLogic,pivotLogic,extendStraightAway, false);
                if (auto.goToParkAfterOuttaking && outtakePixelFinished){
                    intakeTrajectory = auto.autoTrajectories.parkTrajectory(poseEstimate,2);
                    currentState = AutoState.PARK;
                    auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectory);
                }
                else if (outtakePixelFinished){
                    if (numCycles ==1){ // for very first cycle
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackStraightTrajectory(poseEstimate,15,3,0,-27,-17);
                    } else if (numCycles == 2){
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackStraightTrajectory(poseEstimate,15,3,0,-27.5, -20);
                    } else if (numCycles == 3){ // turning into the stacks
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectoryStage(poseEstimate,19,-3,-160,3,0);
                    } else if (numCycles == 4){
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectoryStage(poseEstimate,19,-3,-160,3,0);
                    }
                    auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectory);
                    currentState = AutoState.DROP;
                }
                break;
            case DROP:

                int armHeight = 0;
                if (numCycles == 1){
                    armHeight = 4;
                } else if (numCycles == 2){
                    armHeight = 3;
                } else if (numCycles == 3){
                    armHeight = 5;
                } else if (numCycles == 4) {
                    armHeight = 3;
                }

                if (auto.drop(armHeight, false,110)){
                    currentState = AutoState.GRAB_OFF_STACK;
                }
                break;
            case GRAB_OFF_STACK:
                if (xPosition < -22){
                    auto.autoTrajectories.extendSlidesAroundStage = true;
                }
                int intakeSlidePosition = INTAKE_SLIDE_AUTO_LONG_PRESET;
                boolean extendSlides = true;
                if (numCycles == 3){
                    intakeSlidePosition = 760;
                } else if (numCycles == 4){
                    intakeSlidePosition = 800;
                } if (numCycles >= 3){
                    extendSlides = false;
            }
                if (auto.grabOffStack(numCycleForDifferentLane, true, extendSlides,3, intakeSlidePosition)){
                    currentState = AutoState.AFTER_GRAB_OFF_STACK;
                    Trajectory outtakeTrajectory = null;
                    if (numCycles < 4) {
                        // this is the old spline path
                        outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,16, 175, 4);
                    }
                    if (outtakeTrajectory != null){
                        auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);
                    }
                }
                break;
            case AFTER_GRAB_OFF_STACK:
                if (auto.afterGrabOffStack(2,3, 300,230)){
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


