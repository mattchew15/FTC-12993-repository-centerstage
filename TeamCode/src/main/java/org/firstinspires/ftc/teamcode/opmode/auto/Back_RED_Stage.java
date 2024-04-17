//package org.firstinspires.ftc.teamcode.opmode.auto;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
//import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
//import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;
//
//import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
//import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.*;
//
//@Autonomous(name = "Back Red Stage Auto", group = "Autonomous")
//public class Back_RED_Stage extends LinearOpMode {
//
//    int numCycleForDifferentLane = 0;
//    double delayForYellow = 0; // this is in seconds
//    double endAngleForStacks = -173;
//    boolean didWeFuckingRelocalize = false;
//
//
//    //Accessories
//    AutoSequences auto = new AutoSequences(telemetry,3);
//    LoopTime loopTime = new LoopTime();
//    // this works because the parameters being passed in don't change throughout the opmode
//    // same cycle numbers
//    AutoRail railLogic = new AutoRail(numCycleForDifferentLane,0, auto,true,3);
//    AutoPivot pivotLogic = new AutoPivot(numCycleForDifferentLane,0, auto, telemetry,3);
//    private double railTarget;
//
//
//    enum AutoState {
//        DELAY,
//        PRELOAD_DRIVE,
//        PLACE_AND_INTAKE,
//        AFTER_PURPLE_DRIVE,
//        TRANSFER_PIXEL,
//        OUTTAKE_PIXEL,
//        DROP,
//        GRAB_OFF_STACK,
//        AFTER_GRAB_OFF_STACK,
//        AFTER_GRAB_OFF_STACK_TOP,
//        OUTTAKE_PIXEL_NO_INTAKE_SLIDES,
//        GO_BACK_FOR_YELLOW,
//        GO_BACK_FOR_WHITES,
//        PARK,
//        IDLE,
//        DELAY_BACK,
//        PRELOAD_DRIVE_BACK,
//        PLACE_AND_INTAKE_BACK,
//        PRELOAD_DRIVE_CASE_3,
//        AFTER_PRELOAD_DRIVE_3
//    }
//
//    AutoState currentState;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        frontOrBackAuto = false;
//        SetAuto.setRedAuto();
//        auto.setGamepad1(gamepad1);
//        if (!frontOrBackAuto){
//            isArmDown = true;
//        }
//
//        if (S == -1){
//            MiddleLaneYIntake -= 3.5;
//        }
//
//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            module.clearBulkCache();
//        } //
//
//        auto.initAutoHardware(hardwareMap,this);
//
//        // trajectories that aren't changing should all be here
//        while (!isStarted()) { // initialization loop
//            auto.intializationLoop(!isArmDown);
//            if (teamPropLocation == 1){
//                telemetry.addLine("Front");
//            } else if (teamPropLocation == 2){
//                telemetry.addLine("Middle");
//            } else if (teamPropLocation == 3){
//                telemetry.addLine("Back");
//            }
//            telemetry.addData("S", S);
//            telemetry.update();
//        }
//
//
//        waitForStart();
//        if (isStopRequested()) return;
//        // runs instantly once
//        auto.afterWaitForStart(!frontOrBackAuto, frontOrBackAuto? auto.autoTrajectories.startPoseFront: auto.autoTrajectories.startPoseBack);
//        if (frontOrBackAuto){
//            currentState = AutoState.DELAY;
//        } else {
//            currentState = AutoState.DELAY_BACK;
//        }
//
//        // can set drive constraints here
//        while (opModeIsActive() && !isStopRequested()) {
//            // Reading at the start of the loop
//            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
//                module.clearBulkCache();
//            }
//
//            auto.mainAutoLoop(
//                    currentState == AutoState.OUTTAKE_PIXEL && xPosition > 20,
//                    (currentState == AutoState.GRAB_OFF_STACK && xPosition < -25) || currentState == AutoState.AFTER_GRAB_OFF_STACK || currentState == AutoState.PLACE_AND_INTAKE,
//                    currentState != AutoState.PRELOAD_DRIVE && currentState != AutoState.OUTTAKE_PIXEL);
//
//            autoSequence();
//            loopTime.delta();
//            //telemetry.addData("numCycles", numCycles);
//            telemetry.addData("Preload", auto.cameraHardware.getPreloadYellowPose());
//            telemetry.addData("LoopTime", loopTime.getDt() / 1_000_000);
//            //telemetry.addData("Hz", loopTime.getHz());
//            telemetry.addData("Auto State", currentState);
//
//
//            telemetry.addData("X OFfset", auto.cameraHardware.ROBOT_X);
//            telemetry.addData("Y Offset", auto.cameraHardware.ROBOT_Y);
//
//            telemetry.addData("Target tag", auto.cameraHardware.getTargetTag());
//            telemetry.addData("Num tag we see", auto.cameraHardware.getNumSeenTags());
//            telemetry.addData("Did we fucking relocalize???", didWeFuckingRelocalize);
//
//            telemetry.addData("armHeight", auto.armHeight);
//            //telemetry.addData("intakeSlidePosition", auto.intakeSubsystem.intakeSlidePosition);
//
//
//            // TODO this will tank our looptimes
//          /*  telemetry.addData("IntakeSlideMotor Current", auto.intakeSubsystem.IntakeSlideMotor.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("LiftMotor Current", auto.outtakeSubsystem.LiftMotor.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("PitchMotor Current", auto.outtakeSubsystem.PitchMotor.getCurrent(CurrentUnit.AMPS));*/
//
//
//            telemetry.update();
//
//        }
//        auto.storePoseEndAuto(poseEstimate);
//    }
//
//    public void autoSequence(){
//
//        if (auto.goToPark(currentState == AutoState.IDLE,2)){
//            currentState = AutoState.IDLE;
//        }
//
//        switch (currentState) {
//            case DELAY_BACK:
//                if(auto.delayState(!isArmDown? 100:0)){
//                    currentState = AutoState.PRELOAD_DRIVE_BACK;
//                    if (teamPropLocation == 1){
//                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive1);
//                    } else if (teamPropLocation == 2){
//                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive2);
//                    } else if (teamPropLocation == 3){
//                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive3);
//                    }
//                }
//                break;
//            case PRELOAD_DRIVE_BACK:
//                if(auto.preloadDriveStateBack()){
//                    currentState = AutoState.PLACE_AND_INTAKE_BACK;
//                }
//                break;
//            case PLACE_AND_INTAKE_BACK:
//                if (auto.placeAndIntakeBackSTage(0.8)){
//                    currentState = AutoState.DROP;
//                }
//                break;
//
//            case DELAY:
//                if(auto.delayState(0)){
//                    if (teamPropLocation == 1){
//                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive1FrontStage);
//                    } else if (teamPropLocation == 2){
//                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive2FrontStage);
//                    } else if (teamPropLocation == 3){
//                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive3FrontFirst);
//                    }
//                    if (teamPropLocation != 3){
//                        currentState = AutoState.PRELOAD_DRIVE;
//                    } else {
//                        currentState = AutoState.PRELOAD_DRIVE_CASE_3;
//                    }
//
//                }
//                break;
//
//            case PRELOAD_DRIVE_CASE_3:
//                if (auto.preloadDriveState3()){
//                    currentState = AutoState.AFTER_PRELOAD_DRIVE_3;
//                    auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive3FrontSecond);
//                }
//                break;
//
//            case AFTER_PRELOAD_DRIVE_3:
//                if (auto.afterPreloadDriveState3()){
//                    currentState = AutoState.PLACE_AND_INTAKE;
//                    auto.frontThirdCase = true;
//                }
//                break;
//
//            case PRELOAD_DRIVE:
//                if(auto.preloadDriveState(false, true,950, 0.18, teamPropLocation == 2 || teamPropLocation == 3)){
//                    currentState = AutoState.PLACE_AND_INTAKE;
//                }
//                break;
//
//            case PLACE_AND_INTAKE:
//                if (auto.placeAndIntakeFrontMIDTRUSS(350,0.18, true)){
//                    if (auto.goBackForYellowPixel){
//                        currentState = AutoState.GO_BACK_FOR_YELLOW;
//                    } else {
//                        if (auto.GlobalTimer.seconds() > delayForYellow){
//                            Trajectory startDrive = null;
//                            if (teamPropLocation == 2){
//                                startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple2;
//                            } else if (teamPropLocation == 1){
//                                startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple1;
//                            } else if (teamPropLocation == 3){
//                                startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple3;
//                            }
//                            auto.autoTrajectories.drive.followTrajectoryAsync(startDrive);
//                            currentState = AutoState.TRANSFER_PIXEL;
//                        }
//                    }
//                }
//                break;
//
//            case GO_BACK_FOR_YELLOW:
//                if (auto.reExtendSLidesForYellow(1200,0.35)){
//                    if (auto.GlobalTimer.seconds() > delayForYellow){
//                        Trajectory startDrive = null;
//                        if (teamPropLocation == 2){
//                            startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple2;
//                        } else if (teamPropLocation == 1){
//                            startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple1;
//                        } else if (teamPropLocation == 3){
//                            startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple3;
//                        }
//                        auto.autoTrajectories.drive.followTrajectoryAsync(startDrive);
//                        currentState = AutoState.TRANSFER_PIXEL;
//                    }
//                }
//                break;
//
//            case TRANSFER_PIXEL:
//                if (numCycles == 0 && frontOrBackAuto){
//                    if (xPosition>-50 && xPosition<10){
//                        auto.intakeSubsystem.intakeSpin(-1);
//                    }
//                }
//                if (numCycles < 3){
//                    auto.goBackToStack(3,6,-29.3);
//                }
//                if (auto.goBackToStack){
//                    currentState = AutoState.GRAB_OFF_STACK;
//                }
//                if (auto.transferPixel()){
//                    currentState = AutoState.OUTTAKE_PIXEL;
//                }
//                break;
//
//            case OUTTAKE_PIXEL:
//                //outtaking lengths for each cycle
//                double liftTarget = 0; // could cause issues if these stay zero
//                int pitchTarget = 0;
//                int intakeSlideTarget = 330; // pre-extend intake for most cycles
//                Trajectory intakeTrajectory = null;
//                boolean openGrippers = true;
//                boolean extendStraightAway = false;
//                if (xPosition > -7 && numCycles != 0){ // custom extend on the first cycle
//                    auto.autoTrajectories.extendSlidesAroundTruss = true;
//                }
//                if (numCycles == 0){
//                    intakeSlideTarget = 10;
//                    if (xPosition > 8){
//                        auto.outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
//                    }
//                    if (xPosition > 18){ //teamPropLocation != 1? xPosition > 21: teamPropLocation == 2? xPosition > 19:
//                        // stop yellow detection
//                        auto.cameraHardware.pausePreloadProcessor();
//                        railTarget = auto.cameraHardware.getRailTarget(auto.correctedHeading, ticksToInchesSlidesMotor(auto.outtakeSubsystem.liftPosition), auto.outtakeSubsystem.pitchEncoderPosition);
//                        railLogic.setRailTargetFromAprilTag(railTarget);
//                    }
//
//                    pitchTarget = 26; // yellow pixel should be pitched higher
//                    liftTarget = 13;
//                    openGrippers = false;
//                } else if (numCycles == 1)
//                {
//                    pitchTarget = 19;
//                    liftTarget = 27;
//                } else if (numCycles == 2){
//                    pitchTarget = 23;
//                    liftTarget = 30.5;
//                } else if (numCycles == 3){
//                    pitchTarget = 25;
//                    liftTarget = 31;
//                    intakeSlideTarget = 70;
//                } else if (numCycles == 4){
//                    pitchTarget = 27;
//                    liftTarget = 31;
//                    intakeSlideTarget = 70;
//                    auto.goToParkAfterOuttaking = true;
//                }
//                boolean outtakePixelFinished = auto.outtakePixel(auto.correctedHeading,liftTarget,pitchTarget,
//                        intakeSlideTarget,railLogic,pivotLogic,extendStraightAway,
//                        true, false, openGrippers, true);
//                if (auto.goToParkAfterOuttaking && outtakePixelFinished || (teamPropLocation == 1 && numCycles == 3)){ // if team prop location is 1 we don't want more pixels
//                    intakeTrajectory = auto.autoTrajectories.parkTrajectory(poseEstimate,2);
//                    currentState = AutoState.PARK;
//                    auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectory);
//                }
//                else if (outtakePixelFinished){
//
//                    if (numCycles == 1){
//
//                        intakeTrajectory = auto.autoTrajectories.driveBackToDropYellow(poseEstimate,10,5.2);
//                    }
//                    if (numCycles == 2){
//                        intakeTrajectory = auto.autoTrajectories.driveIntoStackStraightTrajectory(new Pose2d(xPosition+1.8,yPosition,headingPosition),22,3,2.3 + S == -1?0:0,-27.5, -20, S == 1? 180:180);
//                    } else if (numCycles == 3 || numCycles == 4){ // turning into the stacks
//                        intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectoryStage(new Pose2d(xPosition+2.7,yPosition,headingPosition),20,-3,endAngleForStacks,3,3.8 + S == -1?-0.9:0,-18);
//                    }
//                    //TODO mental note - if you move the x distance upwards the angle needs to be less and the offset needs to be more for the spline to work properly
//                    /*else if (numCycles == 4){
//                        intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectoryStage(poseEstimate,22,-2,endAngleForStacks,3,0,-23);
//                    }*/
//                    didWeFuckingRelocalize = auto.resetPosWithAprilTags(3);
//
//                    if (intakeTrajectory != null){
//                        auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectory);
//                    }
//
//                    currentState = AutoState.DROP;
//                }
//                break;
//            case DROP:
//                Trajectory intakeTrajectoryAfterDrop;
//                double delayTime = 90;
//                int armHeight = 0;
//                if (numCycles == 1){
//                    if (auto.delay(500)){
//                        auto.outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
//                        if(auto.delay(710)){
//                            auto.outtakeSubsystem.liftToInternalPID(0,0.4);
//                        } // so we don't rely on a drive back
//                    }
//                    if(auto.delay(270) && !frontOrBackAuto){
//                        auto.outtakeSubsystem.liftToInternalPID(0,0.5);
//                    }
//                    delayTime = frontOrBackAuto? (teamPropLocation == 3?830:690):300;
//                    if (frontOrBackAuto){
//                        armHeight = 4;
//                    } else {
//                        armHeight = 6;
//                    }
//                } else if (numCycles == 2){
//                    if (frontOrBackAuto){
//                        armHeight = 1;
//                    } else {
//                        armHeight = 3;
//                    }
//                } else if (numCycles == 3){
//                    armHeight = 6;
//                } else if (numCycles == 4) {
//                    armHeight = 3;
//                }
//
//                if (auto.drop(armHeight, false, delayTime)){
//                    if (numCycles == 1){ // for very first cycle
//                        if (frontOrBackAuto){
//                            if (teamPropLocation == 1){
//                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterYellowStage1);
//                            } else if (teamPropLocation == 2){
//                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterYellowStage2);
//                            } else if (teamPropLocation == 3){
//                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterYellowStage3);
//                            }
//                        } else { // for the back side autos we just run this straight away
//                            if (teamPropLocation == 1){
//                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterBackStage1);
//                            } else if (teamPropLocation == 2){
//                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterBackStage2);
//                            } else if (teamPropLocation == 3){
//                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterBackStage3);
//                            }
//                        }
//                    }
//                    if (auto.GlobalTimer.seconds() > 25.1){
//                        auto.parkIfStuck = true; // should force into park before doing another cycle
//                    } else {
//                        currentState = AutoState.GRAB_OFF_STACK;
//                    }
//                }
//                break;
//            case GRAB_OFF_STACK:
//                if ((xPosition < -18) && ((endAngleForStacks - Math.toDegrees(headingPosition)) < 2.8)){ // test to see if this works
//                    auto.autoTrajectories.extendSlidesAroundStage = true;
//                }
//                double delayBeforeRetracting = 200;
//                int intakeSlidePosition = INTAKE_SLIDE_AUTO_LONG_PRESET;
//                boolean extendSlides = false;
//                double xPosSlideThresh = -10;
//                boolean retractSlides = false;
//                double xSplineValue = 7;
//                double yOffset = 3.7;
//                double endTangent = -6;
//                double slideSpeed = 1;
//                if (numCycles == 1){
//                    delayBeforeRetracting = 500;
//                    retractSlides = true;
//                    slideSpeed = 0.8;
//                    if (S == 1? xPosition < 12 :xPosition < 12 ){//xPosition < -17
//                        auto.autoTrajectories.extendSlidesAroundStage = true;
//                    }
//                    /*yOffset = 4;
//                    xSplineValue = 6;*/
//                   /* if (S == 1){
//                        extendSlides = true;
//                    }*/
//                }
//                if (numCycles == 2){
//                    slideSpeed = 0.7;
//                    if (S == 1? xPosition < 12 : xPosition < 12){ //  xPosition < -14
//                        auto.autoTrajectories.extendSlidesAroundStage = true;
//                    }/*if (S == 1){
//                        extendSlides = true;
//                    }*/
//                }
//                if (numCycles == 3 || numCycles == 4){
//
//                    intakeSlidePosition = 850;
//                    xSplineValue = 3;
//                    yOffset = 4;
//                    extendSlides = false;
//                    retractSlides = true;
//                    endTangent = -7;
//                }
//                if (auto.grabOffStack(numCycleForDifferentLane, true, extendSlides,3, intakeSlidePosition, delayBeforeRetracting, xPosSlideThresh, retractSlides, slideSpeed)){
//                    currentState = AutoState.AFTER_GRAB_OFF_STACK;
//
//                    Trajectory outtakeTrajectory = null;
//                 /*   if (numCycles == 1){
//                        outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,18, 171, 4.8,14);
//                    }
//                    if (numCycles == 2){
//                        outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,18, 169, 4.8,14);
//                    } else if (numCycles == 3){
//                        outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,16, 168, 5.4,12);
//                    } else if (numCycles == 4){
//                        outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,16, 168, 5.4,12);
//                    }*/
//
//
//                 /*   if (numCycles >= 3) { // for the longer delay we follow the trajectory after the wait - just so its more consistent hopefully
//                          //outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromAngleTurnEndTrajectory(poseEstimate, 20, 31, endTangent, -yOffset, 3,-3,10);
//                          outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,20, 173, 4.3);
//                       //   auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);
//                      }
//                    //  else{// (numCycles < 4) {
//                    // this is the old spline path
//                    else {
//                          outtakeTrajectory = auto.autoTrajectories.simplifiedOuttakeDrive(poseEstimate, numCycles >= 3 ? 22 : 19, 176.8, endTangent, yOffset, xSplineValue);
//                         }*/
//                    outtakeTrajectory = auto.autoTrajectories.simplifiedOuttakeDrive(poseEstimate, numCycles >= 3 ? 19 : 18, 176.8, endTangent, yOffset, xSplineValue);
//                    //      outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,18, 175, 4);
//                    //     outtakeTrajectory = auto.autoTrajectories.outtakeDriveMiddlePathTrajectory(poseEstimate,18, 175, 4);
//                    //   }
//                    if (outtakeTrajectory != null){
//                        auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);
//                    }
//                }
//                break;
//            case AFTER_GRAB_OFF_STACK:
//                if (auto.afterGrabOffStack(2,3, 365,300)){
//                    currentState = AutoState.TRANSFER_PIXEL;
//
//                }
//                break;
//            case PARK:
//                if (auto.park()){
//                    currentState = AutoState.IDLE;
//                }
//                break;
//            case IDLE:
//                auto.idle();
//                break;
//        }
//
//    }
//}