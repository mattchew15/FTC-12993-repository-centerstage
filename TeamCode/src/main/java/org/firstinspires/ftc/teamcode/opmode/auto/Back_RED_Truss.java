package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.*;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.frontOrBackAuto;

@Autonomous(name = "Back Red Truss Auto", group = "Autonomous")
public class Back_RED_Truss extends LinearOpMode {

    int numCycleForDifferentLane = 0;
    double positiveDriftOffset1 = 0;
    double positiveDriftOffset2 = 0;
    double delayForYellow = 8;
    boolean frontOrBackAuto;

    //Accessories
    AutoSequences auto = new AutoSequences(telemetry,1);
    LoopTime loopTime = new LoopTime();
    // this works because the parameters being passed in don't change throughout the opmode
    // same cycle numbers
    AutoRail railLogic = new AutoRail(numCycleForDifferentLane,0, auto, false, 1);
    AutoPivot pivotLogic = new AutoPivot(numCycleForDifferentLane,0, auto, telemetry,1);

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
        PLACE_AND_INTAKE_BACK
    }

    AutoState currentState;

    @Override
    public void runOpMode() throws InterruptedException {

        frontOrBackAuto = false;
        // sets values for generating trajectories
        SetAuto.setRedAuto();
        auto.setGamepad1(gamepad1);
        if (!frontOrBackAuto){
            isArmDown = true;
        }

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            module.clearBulkCache();
        } //

        // generates trajectories for everything
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
        DriveConstants.MAX_VEL = 50;
        DriveConstants.MAX_ACCEL = 42;
        //auto.cameraHardware.resumePreloadProcessor();

        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)){ // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
                module.clearBulkCache();
            }

            auto.mainAutoLoop(
                    currentState == AutoState.OUTTAKE_PIXEL && xPosition > 20,
                    currentState == AutoState.GRAB_OFF_STACK && xPosition < -24 || currentState == AutoState.AFTER_GRAB_OFF_STACK || currentState == AutoState.PLACE_AND_INTAKE,
                    currentState != AutoState.PRELOAD_DRIVE && currentState != AutoState.OUTTAKE_PIXEL);

            autoSequence();
            loopTime.delta();
            //telemetry.addData("globalTimer", auto.GlobalTimer.seconds());
            //telemetry.addData("Preload", auto.cameraHardware.getPreloadYellowPose());
            // telemetry.addData("Target rail pos", auto.cameraHardware.getRailTarget(auto.correctedHeading,ticksToInchesSlidesMotor(auto.outtakeSubsystem.liftPosition), auto.outtakeSubsystem.pitchEncoderPosition));
            //telemetry.addData("numCycles", numCycles);
            //     telemetry.addData("outtakeDistanceSensorValue", auto.outtakeSubsystem.outtakeDistanceSensorValue);
            //telemetry.addData("xPosition", xPosition);
            //    telemetry.addData("yPosition", yPosition);
            //telemetry.addData("headingPosition", headingPosition);
            telemetry.addData("arm height", auto.armHeight);
            telemetry.addData("LoopTime", loopTime.getDt() / 1_000_000);
            //telemetry.addData("Hz", loopTime.getHz());
            telemetry.addData("Auto State", currentState);
            telemetry.addData("intakeSlidePosition", auto.intakeSubsystem.intakeSlidePosition);

            telemetry.update();

        }
        auto.storePoseEndAuto(poseEstimate);
    }

    public void autoSequence(){
        if (auto.goToPark(currentState == AutoState.IDLE,1)){
            currentState = AutoState.IDLE;
        }
        switch (currentState) {
            case DELAY_BACK:
                if(auto.delayState(0)){
                    currentState = AutoState.PRELOAD_DRIVE_BACK;
                    if (teamPropLocation == 1){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive1);
                        telemetry.addLine("left");
                    } else if (teamPropLocation == 2){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive2);
                        telemetry.addLine("center");
                    } else if (teamPropLocation == 3){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive3);
                        telemetry.addLine("right");
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
                if(auto.preloadDriveState(true, false,1000,1, true)){
                    currentState = AutoState.PLACE_AND_INTAKE;
                }
                break;

            case PLACE_AND_INTAKE:
                if (auto.placeAndIntakeFrontMIDTRUSS(500,1, true)){
                    if (auto.goBackForYellowPixel){
                        currentState = AutoState.GO_BACK_FOR_YELLOW;
                    } else{
                        Trajectory startDrive = null;
                        if (teamPropLocation == 2){
                            startDrive = auto.autoTrajectories.firstDriveThroughTrussAfterPurple2;
                        } else if (teamPropLocation == 1){
                            startDrive = auto.autoTrajectories.firstDriveThroughTrussAfterPurple1;
                        } else if (teamPropLocation == 3){
                            startDrive = auto.autoTrajectories.firstDriveThroughTrussAfterPurple3;
                        }
                        auto.autoTrajectories.drive.followTrajectoryAsync(startDrive);
                        currentState = AutoState.TRANSFER_PIXEL;
                    }
                }
                break;

            case GO_BACK_FOR_YELLOW:
                if (auto.reExtendSLidesForYellow(1500,0.35)){
                    if (auto.GlobalTimer.seconds() > delayForYellow){ // the robot takes 3 seconds to go deposit yellow after this
                        Trajectory startDrive = null;
                        if (teamPropLocation == 2){
                            startDrive = auto.autoTrajectories.firstDriveThroughTrussAfterPurple2;
                        } else if (teamPropLocation == 1){
                            startDrive = auto.autoTrajectories.firstDriveThroughTrussAfterPurple1;
                        } else if (teamPropLocation == 3){
                            startDrive = auto.autoTrajectories.firstDriveThroughTrussAfterPurple3;
                        }
                        auto.autoTrajectories.drive.followTrajectoryAsync(startDrive);
                        currentState = AutoState.TRANSFER_PIXEL;
                    }
                }
                break;

            case TRANSFER_PIXEL:
                if (auto.goBackToStack){
                    currentState = AutoState.GRAB_OFF_STACK;
                }
                if (auto.transferPixel(numCycles==0?false:true)){
                    currentState = AutoState.OUTTAKE_PIXEL;
                    auto.extendOuttake = true;
                }
                break;

            case OUTTAKE_PIXEL:
                //outtaking lengths for each cycle

                double liftTarget = 0; // could cause issues if these stay zero
                int pitchTarget = 0;
                int intakeSlideTarget = 90; // pre-extend a little
                double railTarget;
                boolean openGrippers = true;
                boolean extendStraightAway = false;
                if (xPosition > 0 && numCycles != 0){ // custom extend on the first cycle
                    auto.autoTrajectories.extendSlidesAroundTruss = true;
                }
                if (numCycles == 0){ // for very first cycle
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
                    liftTarget = 13.4;
                    openGrippers = false;
                    //extendStraightAway = false;
                } else if (numCycles == 1){
                    pitchTarget = 23;
                    liftTarget = 25;
                } else if (numCycles == 2){
                    pitchTarget = 26;
                    liftTarget = 27;
                    if (frontOrBackAuto){ // go for only 2+5 for front autos, 2+6 for back :)
                        auto.goToParkAfterOuttaking = true;
                    }
                } else if (numCycles == 3){
                    pitchTarget = 27;
                    liftTarget = 28;
                    auto.goToParkAfterOuttaking = true;
                }
                boolean outtakePixelFinished = auto.outtakePixel(auto.correctedHeading,liftTarget,pitchTarget,intakeSlideTarget,railLogic,pivotLogic,extendStraightAway,false, false, openGrippers, true);

                if (outtakePixelFinished){
                    currentState = AutoState.DROP;
                    Trajectory intakeTrajectoryCycle = null;
                    if (numCycles == 1){
                        intakeTrajectoryCycle = auto.autoTrajectories.driveBackToDropYellow(poseEstimate,10,3.5);
                        // intakeTrajectoryCycle = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectory(poseEstimate,22,5,148,1,0.9);
                        // this cycle runs on the drop case now
                    } else if (numCycles == 2){
                        intakeTrajectoryCycle = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectory(poseEstimate,25,5,153.7,1,positiveDriftOffset1, -36);
                        //auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectoryCycle2);
                    } else if (numCycles == 3){
                        intakeTrajectoryCycle = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectory(poseEstimate,25,6.5,145,1,positiveDriftOffset2,-36);
                        //auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectoryCycle2);
                    }
                    auto.resetPosWithAprilTags(1,S == -1? false:true);

                    if(intakeTrajectoryCycle!= null){
                        auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectoryCycle);
                    }
                }


                break;
            case DROP:
                int armHeight = 5;
                double delayTime = 160;
                if (numCycles == 1){
                    if (frontOrBackAuto){
                        armHeight = 4;
                    } else{
                        armHeight = 5;
                    }
                    if (auto.delay(600)){
                        auto.outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                    }
                    delayTime = frontOrBackAuto? 800:350;
                } else if (numCycles == 2){
                    if (frontOrBackAuto){
                        armHeight = 1;
                    } else{
                        armHeight = 3;
                    }
                } else if (numCycles == 3){
                    armHeight = 5;
                } else if (numCycles == 4) {
                    armHeight = 3;
                }
                if (auto.drop(armHeight,auto.goToParkAfterOuttaking, delayTime)){
                    if (numCycles == 1){ // for very first cycle
                        if (frontOrBackAuto){
                            if (teamPropLocation == 1){
                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterYellowTruss1);
                            } else if (teamPropLocation == 2){
                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterYellowTruss2);
                            } else if (teamPropLocation == 3){
                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterYellowTruss3);
                            }
                        } else {
                            if (teamPropLocation == 1){
                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterBackTruss1);
                            }  else if (teamPropLocation == 2){
                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterBackTruss2);
                            } else if (teamPropLocation == 3){
                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterBackTruss3);
                            }
                        }
                    }
                    currentState = AutoState.GRAB_OFF_STACK;
                    if (auto.goToParkAfterOuttaking){
                        Trajectory intakeTrajectory = auto.autoTrajectories.parkTrajectory(poseEstimate,1);
                        currentState = AutoState.PARK;
                        auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectory);
                    }
                }
                break;
            case GRAB_OFF_STACK:
                if (xPosition < -25){
                    auto.autoTrajectories.extendSlidesAroundTruss = true;
                }
                double delayBeforeRetracting = 260;
                if (numCycles == 1){
                    delayBeforeRetracting = 500;
                }
                if (auto.grabOffStackTruss(delayBeforeRetracting)){
                    currentState = AutoState.AFTER_GRAB_OFF_STACK;
                }
                break;
            case AFTER_GRAB_OFF_STACK:
                //auto.lockTo(auto.autoTrajectories.trussSideStackCoordinate);
                if (auto.afterGrabOffStack(2,2,300,200)){
                    if (!auto.intakeSubsystem.pixelsInIntake()){
                        currentState = AutoState.GO_BACK_FOR_WHITES;
                    } else {
                        currentState = AutoState.TRANSFER_PIXEL;
                        Trajectory outtakeTrajectory;
                        outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromAngleTurnEndTrajectory(poseEstimate, 18, 29.8, 5, 6, 1,1.8,9);
                        auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);
                    }
                }
                break;
            case GO_BACK_FOR_WHITES:
                if(auto.reExtendSlidesForTrussSide()){
                    currentState = AutoState.TRANSFER_PIXEL;
                    Trajectory outtakeTrajectory;
                    outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromAngleTurnEndTrajectory(poseEstimate,18,29.8,5,6, 1,1.8,9);
                    auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);
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


