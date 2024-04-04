package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

import static org.firstinspires.ftc.teamcode.opmode.auto.Front_RED_Truss.AutoState.GRAB_OFF_STACK;
import static org.firstinspires.ftc.teamcode.opmode.auto.Front_RED_Truss.AutoState.OUTTAKE_PIXEL;
import static org.firstinspires.ftc.teamcode.opmode.auto.Front_RED_Truss.AutoState.TRANSFER_PIXEL;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.*;

@Autonomous(name = "Front Red Truss", group = "Autonomous")
public class Front_RED_Truss extends LinearOpMode {

    int numCycleForDifferentLane = 0;

    //Accessories
    AutoSequences auto = new AutoSequences(telemetry);
    LoopTime loopTime = new LoopTime();
    // this works because the parameters being passed in don't change throughout the opmode
    // same cycle numbers
    AutoRail railLogic = new AutoRail(numCycleForDifferentLane,0, auto, false, 1);
    AutoPivot pivotLogic = new AutoPivot(numCycleForDifferentLane,0, auto, telemetry,1);

    Trajectory intakeTrajectoryCycle1;
    Trajectory intakeTrajectoryCycle2;

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


        DriveConstants.MAX_VEL = 53;

        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)){ // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
                module.clearBulkCache();
            }

            auto.mainAutoLoop(
                    currentState == OUTTAKE_PIXEL && xPosition > 20,
                    currentState == GRAB_OFF_STACK || currentState == AutoState.AFTER_GRAB_OFF_STACK || currentState == AutoState.PLACE_AND_INTAKE,
                    currentState != AutoState.PRELOAD_DRIVE && currentState != OUTTAKE_PIXEL);

            autoSequence();
            loopTime.delta();
            telemetry.addData("Preload", auto.cameraHardware.getPreloadYellowPose());
            telemetry.addData("Target rail pos", auto.cameraHardware.getRailTarget());
            telemetry.addData("numCycles", numCycles);
            telemetry.addData("outtakeDistanceSensorValue", auto.outtakeSubsystem.outtakeDistanceSensorValue);
            telemetry.addData("xPosition", xPosition);
            telemetry.addData("yPosition", yPosition);
            telemetry.addData("headingPosition", headingPosition);
            telemetry.addData("LoopTime", loopTime.getDt() / 1_000_000);
            //telemetry.addData("Hz", loopTime.getHz());
            telemetry.addData("Auto State", currentState);
            telemetry.addData("intakeSlidePosition", auto.intakeSubsystem.intakeSlidePosition);

            telemetry.update();

        }
        auto.storePoseEndAuto(poseEstimate);
    }

    public void autoSequence(){
        auto.goToPark(currentState == AutoState.IDLE,1);
        switch (currentState) {
            case DELAY:
                if(auto.delayState(0)){
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
                if(auto.preloadDriveState(true, true,1000,0.38)){
                    currentState = AutoState.PLACE_AND_INTAKE;
                }
                break;
            case PLACE_AND_INTAKE:
                if (auto.placeAndIntakeFrontMIDTRUSS(500,0.38)){
                    Trajectory startDrive = auto.autoTrajectories.firstDriveThroughTrussAfterPurple2;
                    auto.autoTrajectories.drive.followTrajectoryAsync(startDrive);
                    currentState = TRANSFER_PIXEL;
                }
                break;

            case TRANSFER_PIXEL:
                if (auto.goBackToStack){
                    currentState = GRAB_OFF_STACK;
                }
                if (auto.transferPixel()){
                    currentState = OUTTAKE_PIXEL;
                }
                break;

            case OUTTAKE_PIXEL:
                //outtaking lengths for each cycle

                double liftTarget = 0; // could cause issues if these stay zero
                int pitchTarget = 0;
                int intakeSlideTarget = 90; // pre-extend a little
                double railTarget;
                if (numCycles == 0){ // for very first cycle
                    railTarget = auto.cameraHardware.getRailTarget();
                    railLogic.setRailTargetFromAprilTag(railTarget);
                    pitchTarget = 23;
                    liftTarget = 19.4;
                } else if (numCycles == 1){
                    pitchTarget = 22;
                    liftTarget = 27;
                } else if (numCycles == 2){
                    pitchTarget = 24;
                    liftTarget = 27;
                    auto.goToParkAfterOuttaking = true;
                }
                boolean outtakePixelFinished = auto.outtakePixel(auto.correctedHeading,liftTarget,pitchTarget,intakeSlideTarget,railLogic,pivotLogic,false,false);

                if (outtakePixelFinished){
                    auto.cameraHardware.pauseBackWebcam();
                    currentState = AutoState.DROP;
                    if (numCycles == 1){
                        intakeTrajectoryCycle1 = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectory(poseEstimate,22,5,148,1,0.9);
                        auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectoryCycle1);
                    } else if (numCycles == 2){
                        intakeTrajectoryCycle2 = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectory(poseEstimate,22,5,145,1,2.6);
                        auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectoryCycle2);
                    }

                }


                break;
            case DROP:
                int armHeight = 0;
                if (numCycles == 1){
                    armHeight = 4;
                } else if (numCycles == 2){
                    armHeight = 1;
                } else if (numCycles == 3){
                    armHeight = 5;
                } else if (numCycles == 4) {
                    armHeight = 3;
                }
                if (auto.drop(armHeight,numCycles != 1, 110)){
                    currentState = AutoState.GRAB_OFF_STACK;
                    if (auto.goToParkAfterOuttaking){
                        Trajectory intakeTrajectory = auto.autoTrajectories.parkTrajectory(poseEstimate,1);
                        currentState = AutoState.PARK;
                        auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectory);
                    }
                }
                break;
            case GRAB_OFF_STACK:
                if (xPosition < -24.5){
                    auto.autoTrajectories.extendSlidesAroundTruss = true;
                }

                if (auto.grabOffStackTruss()){
                    currentState = AutoState.AFTER_GRAB_OFF_STACK;
                    Trajectory outtakeTrajectory;
                    outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromAngleTurnEndTrajectory(poseEstimate,18,29.8,4,4, 1);
                    auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);
                }
                break;
            case AFTER_GRAB_OFF_STACK:
                if (auto.afterGrabOffStack(2,2,800,700)){
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


