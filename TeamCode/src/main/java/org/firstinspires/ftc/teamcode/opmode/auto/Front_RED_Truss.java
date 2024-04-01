package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

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
    AutoRail railLogic = new AutoRail(numCycleForDifferentLane,0, auto, false);
    AutoPivot pivotLogic = new AutoPivot(numCycleForDifferentLane,0, auto);


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
                    currentState == AutoState.OUTTAKE_PIXEL && xPosition > 24,
                    currentState == AutoState.GRAB_OFF_STACK || currentState == AutoState.AFTER_GRAB_OFF_STACK || currentState == AutoState.PLACE_AND_INTAKE,
                    currentState != AutoState.PRELOAD_DRIVE && currentState != AutoState.OUTTAKE_PIXEL);


            autoSequence();
            loopTime.delta();
            telemetry.addData("LoopTime", loopTime.getDt() / 1_000_000);
            telemetry.addData("Hz", loopTime.getHz());
            telemetry.addData("Auto State", currentState);
            telemetry.addData("intakeSlidePosition", auto.intakeSubsystem.intakeSlidePosition);

            telemetry.update();

        }
    }

    public void autoSequence(){
        auto.goToPark(currentState == AutoState.IDLE,2);
        switch (currentState) {
            case DELAY:
                if(auto.delayState(0)){
                    currentState = AutoState.PRELOAD_DRIVE;
                }
                break;
            case PRELOAD_DRIVE:
                if(auto.preloadDriveState()){
                    currentState = AutoState.PLACE_AND_INTAKE;
                }
                break;
            case PLACE_AND_INTAKE:
                Trajectory startDrive = null;
                if(teamPropLocation == 2){
                    startDrive = auto.autoTrajectories.firstDriveThroughTrussAfterPurple2; //
                }

                if (auto.placeAndIntakeFrontMIDTRUSS(startDrive)){
                    currentState = AutoState.TRANSFER_PIXEL;
                }
                break;

            case TRANSFER_PIXEL:
                /*
                if (auto.goBackToStack){
                    currentState = AutoState.GRAB_OFF_STACK;
                }
                 */
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
                if (numCycles == 0){ // for very first cycle
                    pitchTarget = 18;
                    liftTarget = 25;
                } else if (numCycles == 1){
                    pitchTarget = 18;
                    liftTarget = 30;
                    intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectory(poseEstimate,12,4.4,160,1);
                } else if (numCycles == 2){
                    intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectory(poseEstimate,12,4.4,160,1);
                } else if (numCycles == 3){
                    auto.goToParkAfterOuttaking = true;
                    intakeTrajectory = auto.autoTrajectories.parkTrajectory(poseEstimate,1);
                }

                boolean outtakePixelFinished = auto.outtakePixel(auto.correctedHeading,liftTarget,pitchTarget,intakeSlideTarget,intakeTrajectory,railLogic,pivotLogic);
                if (auto.goToParkAfterOuttaking && outtakePixelFinished){
                    currentState = AutoState.PARK;
                }
                else if (outtakePixelFinished){
                    currentState = AutoState.DROP;
                }
                break;
            case DROP:

                int armHeight = 0;
                if (numCycles == 1){
                    armHeight = 4;
                } else if (numCycles == 2){
                    armHeight = 1;
                }

                if (auto.drop(armHeight)){
                    currentState = AutoState.DROP;
                }
                break;
            case GRAB_OFF_STACK:
                Trajectory outtakeTrajectory;
                outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromAngleTurnEndTrajectory(poseEstimate,12,5,4, 1);

                // we don't want to pre-Extend our slides at all here
                boolean grabOffStackFinished = auto.grabOffStack(numCycleForDifferentLane, false,false, outtakeTrajectory);
                if (grabOffStackFinished){
                    currentState = AutoState.AFTER_GRAB_OFF_STACK;
                }
                break;
            case AFTER_GRAB_OFF_STACK:
                // this will always run the longer delay time
                if (auto.afterGrabOffStack(2,0,250,120)){
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


