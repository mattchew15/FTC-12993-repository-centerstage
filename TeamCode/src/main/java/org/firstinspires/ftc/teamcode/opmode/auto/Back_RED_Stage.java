
package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.teleop.SimplicityDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.CameraHardware;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;
import org.firstinspires.ftc.teamcode.system.vision.YCrCbRedTeamPropDetectorPipeline;

import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.MiddleLaneYDeposit;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.headingPosition;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.poseEstimate;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.xPosition;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.yPosition;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import static org.firstinspires.ftc.teamcode.system.vision.YCrCbRedTeamPropDetectorPipeline.RED_POSITION;

import android.provider.Settings;
import android.view.ViewTreeObserver;

@Autonomous(name = "Back Red Stage Auto", group = "Autonomous")
public class Back_RED_Stage extends LinearOpMode {

    // class members
    ElapsedTime GlobalTimer;
    double autoTimer;

    int numCycles;
    int teamPropLocation;
    double correctedHeading;
    int timesIntoStack;
    int liftTarget;
    boolean goToPark;

    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    CameraHardware cameraHardware = new CameraHardware();
    AutoTrajectories autoTrajectories;
    //Accessories
    LoopTime loopTime = new LoopTime();


    enum AutoState {
        DELAY,
        PRELOAD_DRIVE,
        OUT_AFTER_PRELOAD_DRIVE,
        DROP_AFTER_PRELOAD,
        OUTTAKE_PIXEL,
        DROP,
        GRAB_OFF_STACK,
        AFTER_GRAB_OFF_STACK,
        TRANSFER_PIXEL,
        OUTTAKE_PIXEL_NO_INTAKE_SLIDES,
        PARK,
        IDLE,
    }


    AutoState currentState;
    Pose2d poseEstimate;

    // Define our start pose

    @Override
    public void runOpMode() throws InterruptedException {

        autoTrajectories = new AutoTrajectories(); // road drive class
        SetAuto.setRedAuto();


        // initialize hardware
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } //

        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        cameraHardware.initWebcam(hardwareMap);
        autoTrajectories.init(hardwareMap);

        // functions runs on start

        // Set inital pose

        // trajectories that aren't changing should all be here

        while (!isStarted()) { // initialization loop
            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
            if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.LEFT){
                telemetry.addLine("left");
                teamPropLocation = 1;
            } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.CENTER){
                telemetry.addLine("center");
                teamPropLocation = 2;
            } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.RIGHT){
                telemetry.addLine("right");
                teamPropLocation = 3;
            }

            telemetry.update();
        }


        waitForStart();
        if (isStopRequested()) return;


        // runs instantly once
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        autoTimer = GlobalTimer.milliseconds();

        goToPark = false;

        intakeSubsystem.intakeHardwareSetup();
        outtakeSubsystem.hardwareSetup();
        outtakeSubsystem.encodersReset();
        intakeSubsystem.intakeSlideMotorEncodersReset();

        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop
        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY); // don't touch this at all in auto
        autoTimer = 0;
        numCycles = 0;
        outtakeSubsystem.outtakeDistanceSensorValue = 100; // so that it doesn't do funky stuff
        autoTrajectories.drive.setPoseEstimate(autoTrajectories.startPoseBack);

        currentState = AutoState.DELAY;
        cameraHardware.closeWebcam(); // reduces loop times

        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            outtakeSubsystem.outtakeReads(currentState == AutoState.OUTTAKE_PIXEL && xPosition > 24); // might need to change this
            intakeSubsystem.intakeReads(currentState == AutoState.GRAB_OFF_STACK || currentState == AutoState.AFTER_GRAB_OFF_STACK);
            poseEstimate = autoTrajectories.drive.getPoseEstimate();

            // Print pose to telemetry
            loopTime.updateLoopTime(telemetry);
            telemetry.addData("distance sensor value", outtakeSubsystem.outtakeDistanceSensorValue);
            telemetry.addData ("times into stack", timesIntoStack);
            telemetry.addData("intakeMotorVelocity", intakeSubsystem.intakeVelocity);
            telemetry.addData("limitswitch", intakeSubsystem.frontLimitSwitchValue);
            telemetry.addData("s",S);
            telemetry.addData("numCycles", numCycles);
            telemetry.addData("Auto State", currentState);
            telemetry.addLine("");
            telemetry.addData("liftPosition", outtakeSubsystem.liftPosition);
            telemetry.addData("pitchPosition", outtakeSubsystem.pitchPosition);
            telemetry.addData("intakeSlidePosition", intakeSubsystem.intakeSlidePosition);
            autoSequence();


            xPosition = poseEstimate.getX();              // is in globals rn - might not work idk
            yPosition = poseEstimate.getY();
            headingPosition = poseEstimate.getHeading();

            telemetry.addData("x Position", xPosition);
            telemetry.addData("y Position", yPosition);
            telemetry.addData("heading", headingPosition);
            correctedHeading = angleWrap(Math.toDegrees(headingPosition) - 180);


            autoTrajectories.drive.update();
            telemetry.update();
        }

    }

    public void autoSequence(){
        if ((GlobalTimer.milliseconds() > 28200) && goToPark && currentState != AutoState.IDLE){
            goToPark = false;
            currentState = AutoState.PARK;
            autoTrajectories.park(poseEstimate,2);
        }
        switch (currentState) {
            case DELAY:
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.UPRIGHT);
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                outtakeSubsystem.pitchToInternalPID(570,1);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                outtakeSubsystem.miniTurretPointToBackdrop(correctedHeading);
                if (GlobalTimer.milliseconds() - autoTimer > 500){
                    autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                    currentState = AutoState.PRELOAD_DRIVE;
                    if (teamPropLocation == 1){
                        autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive1);
                    } else if (teamPropLocation == 2){
                        autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive2);
                    } else if (teamPropLocation == 3){
                        autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive3);
                    }
                }
                break;
            case PRELOAD_DRIVE:
                if (GlobalTimer.milliseconds() - autoTimer > 300) {
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_DOWN);
                }
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                if (!autoTrajectories.drive.isBusy()){
                    autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                    currentState = AutoState.OUT_AFTER_PRELOAD_DRIVE;
                }
                break;

            case OUT_AFTER_PRELOAD_DRIVE:
                if (teamPropLocation == 1){
                    intakeSubsystem.intakeSlideTo(640, intakeSubsystem.intakeSlidePosition,1);
                    liftTarget = 550;
                    if (GlobalTimer.milliseconds()- autoTimer > 20){
                        outtakeSubsystem.pitchToInternalPID(PITCH_LOW_DEGREE_TICKS,1);
                    }
                    outtakePreload(poseEstimate);
                } else if (teamPropLocation == 2){
                    intakeSubsystem.intakeSlideTo(370, intakeSubsystem.intakeSlidePosition,1);
                    liftTarget = 450;
                    if (GlobalTimer.milliseconds()- autoTimer > 20){
                        outtakeSubsystem.pitchToInternalPID(PITCH_LOW_DEGREE_TICKS,1);
                    }
                    outtakePreload(poseEstimate);
                } else if (teamPropLocation == 3){
                    intakeSubsystem.intakeSlideTo(250, intakeSubsystem.intakeSlidePosition,1);
                    liftTarget = 300;
                    if (GlobalTimer.milliseconds()- autoTimer > 20){
                        outtakeSubsystem.pitchToInternalPID(PITCH_LOW_DEGREE_TICKS,1);
                    }
                    if (BLUE_AUTO){
                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
                    }
                    else {
                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
                    }
                    outtakePreload(poseEstimate);
                }
                if (intakeSubsystem.intakeSlideTargetReached() && outtakeSubsystem.liftTargetReached()){
                    if (GlobalTimer.milliseconds() - autoTimer > 200) {
                        currentState = AutoState.DROP_AFTER_PRELOAD;
                        autoTimer = GlobalTimer.milliseconds();
                    }
                } else {
                    autoTimer = GlobalTimer.milliseconds();
                }
                break;

            case DROP_AFTER_PRELOAD:
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP);
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                if (GlobalTimer.milliseconds() - autoTimer > 200){
                    intakeSubsystem.intakeSpin(0);
                    intakeSubsystem.intakeSlideInternalPID(0,1); // retract the slides
                    if (GlobalTimer.milliseconds() - autoTimer > 500){
                        outtakeSubsystem.liftTo(0, outtakeSubsystem.liftPosition, 1);
                        outtakeSubsystem.pitchToInternalPID(0,1);
                        if (intakeSubsystem.intakeSlidePosition < 10 && outtakeSubsystem.liftPosition < 10){
                            autoTimer = GlobalTimer.milliseconds();
                            autoTrajectories.driveIntoStackStageFromMiddlePathStraightEnd(poseEstimate,10, 27);
                            currentState = AutoState.GRAB_OFF_STACK;
                            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP);
                        }
                    }
                } else {
                    intakeSubsystem.intakeSpin(-0.18);
                }


                break;

            // might need to make multiple of these states depending on the trajectory it follows -- again making methods would help alot with this
            case GRAB_OFF_STACK: // put each state in a method in antoher class and pass in timer variable

                outtakeSubsystem.pitchToInternalPID(SIXTY_DEGREE_TICKS, 1);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
                outtakeSubsystem.liftToInternalPID(0,1);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);


                if (xPosition < 6){ //
                    intakeSubsystem.intakeSlideTo(700, intakeSubsystem.intakeSlidePosition,1);
                    if (xPosition < -12) {

                        intakeSubsystem.intakeSpin(1);
                        if (!autoTrajectories.drive.isBusy()){ // do stuff with sensor to make better

                            currentState = AutoState.AFTER_GRAB_OFF_STACK;
                            autoTimer = GlobalTimer.milliseconds();

                            autoTrajectories.outtakeDriveFromStraightTurnEndStageV2(poseEstimate,16,157, 4);

                        }
                    }
                }
                break;

            // might need to make another state for the colour sensors similar to teleop
            case AFTER_GRAB_OFF_STACK:
                if (GlobalTimer.milliseconds() - autoTimer > 0){
                    if (numCycles == 1 || numCycles == 4){
                        intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                    }
                    if (GlobalTimer.milliseconds() - autoTimer > 200){
                        autoTimer = GlobalTimer.milliseconds(); // resets timer
                        currentState = AutoState.TRANSFER_PIXEL;
                        intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.HOLDING);
                    }
                }
                break;
            case TRANSFER_PIXEL:
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                intakeSubsystem.intakeSlideInternalPID(-6,1);
                if (GlobalTimer.milliseconds() - autoTimer > 100){ // time for pixel holder to close
                    intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP);

                    if ((GlobalTimer.milliseconds() - autoTimer > 600) && timesIntoStack < 1){
                        if (!intakeSubsystem.pixelsInIntake()){
                            currentState = AutoState.GRAB_OFF_STACK;
                            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                            intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                            intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.OPEN);
                            timesIntoStack += 1;
                            autoTimer = GlobalTimer.milliseconds();
                            autoTrajectories.goBackToStack(poseEstimate,7,3);

                        }
                    }

                    if (GlobalTimer.milliseconds() - autoTimer > 500){ // also time for pixel holder to close

                        if (intakeSubsystem.intakeSlidePosition < 5){
                            intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.TRANSFER);
                            intakeSubsystem.intakeSpin(0.7);
                            if (intakeSubsystem.intakeSlidePosition < 2){
                                currentState = AutoState.OUTTAKE_PIXEL;
                                autoTimer = GlobalTimer.milliseconds();
                            }
                        } else {
                            intakeSubsystem.intakeSpin(0.6);
                        }
                    } else {
                        intakeSubsystem.intakeSpin(-1);
                    }
                } else {
                    intakeSubsystem.intakeSpin(0.9); // this doesn't happen for very long
                }

                break;

            case OUTTAKE_PIXEL:
                if (GlobalTimer.milliseconds() - autoTimer > 500 ){ // wait for chute to go in properly
                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                    /*
                    if(numCycles == 0 && GlobalTimer.milliseconds() - autoTimer > 530){
                        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                    }
                     */
                    if (GlobalTimer.milliseconds() - autoTimer > 630){ // time for grippers to close
                        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);

                    } else if (numCycles == 0){
                        //outtakeSubsystem.liftTo(725,outtakeSubsystem.liftPosition,1);
                        outtakeSubsystem.updateLiftTargetProfile(650);
                        outtakeSubsystem.pitchToInternalPID(980,1);
                    } else if (numCycles == 1){
                        outtakeSubsystem.updateLiftTargetProfile(734);
                        //outtakeSubsystem.liftTo(738,outtakeSubsystem.liftPosition,1);
                        outtakeSubsystem.pitchToInternalPID(850,1);
                    } else if (numCycles == 2){
                        outtakeSubsystem.updateLiftTargetProfile(780);
                        //outtakeSubsystem.liftTo(738,outtakeSubsystem.liftPosition,1);
                        outtakeSubsystem.pitchToInternalPID(900,1);
                    }
                    outtakeSubsystem.profileLiftCalculate();
                    if (GlobalTimer.milliseconds() - autoTimer > 730){
                        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_DOWN); // slides go out before arm so transfer is good
                        if (GlobalTimer.milliseconds() - autoTimer > 920){ // once chute is down
                            outtakeSubsystem.miniTurretPointToBackdrop(correctedHeading);

                            if (BLUE_AUTO){
                                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
                            }
                            else {
                                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
                            }

                            if (!autoTrajectories.drive.isBusy() || (outtakeSubsystem.outtakeDistanceSensorValue < 6.7 && numCycles != 0)) { // //  || outtakeSubsystem.outtakeDistanceSensorValue < 8.7
                                // line above determines when we drop the pixels
                                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                                outtakeSubsystem.outtakeDistanceSensorValue = 100; // so that it doesn't do funky stuff
                                numCycles += 1;
                                timesIntoStack = 0;
                                autoTimer = GlobalTimer.milliseconds();

                                if (numCycles == 1){
                                    autoTrajectories.driveIntoStackStraightAfterAngledOuttakeSTAGE(poseEstimate,18); // might cause an issue here
                                    currentState = AutoState.DROP;
                                } else if (numCycles == 2){
                                    currentState = AutoState.PARK;
                                    autoTrajectories.park(poseEstimate,2);
                                    //autoTrajectories.driveIntoStackStageFromMiddlePathStraightEnd(poseEstimate,8);
                                    //currentState = AutoState.DROP;
                                } else if (numCycles == 3){
                                    currentState = AutoState.PARK;
                                    autoTrajectories.park(poseEstimate,2);
                                } else if (numCycles == 4){
                                    currentState = AutoState.PARK;
                                    autoTrajectories.park(poseEstimate,2);
                                }
                            }
                        }
                    }
                } else {
                    intakeSubsystem.intakeSpin(0.5);
                }
                break;
            case DROP:
                // waits for robot to move away far enough - minimize this
                if (GlobalTimer.milliseconds() - autoTimer > 100){
                    outtakeSubsystem.liftToInternalPID(-6,1);
                    outtakeSubsystem.pitchToInternalPID(SIXTY_DEGREE_TICKS,1); // SIXTY_DEGREE_TICKS = 0 lmao
                    if (GlobalTimer.milliseconds() - autoTimer > 200){ // not knock pixels off the backdrop
                        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                        //     intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                        if (numCycles == 1){
                            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE);
                        } else if (numCycles == 2){
                            // make intake arm height correct
                            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP);
                        } else if (numCycles == 3){
                            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE);
                        }
                        currentState = AutoState.GRAB_OFF_STACK;
                        autoTimer = GlobalTimer.milliseconds();
                        intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.OPEN);

                    }

                }
                break;
            case PARK:

                if (GlobalTimer.milliseconds() - autoTimer > 200){
                    currentState = AutoState.IDLE; // doesn't have to drive anywhere, already in position hopefully
                }

                break;

            case IDLE:
                telemetry.addLine("WWWWWWWWWWW");
                intakeSubsystem.intakeSlideInternalPID(-6,1);
                outtakeSubsystem.liftToInternalPID(-8,1);
                outtakeSubsystem.pitchToInternalPID(SIXTY_DEGREE_TICKS, 1);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                break;

        }
    }
    void outtakePreload(Pose2d poseEstimate){
        outtakeSubsystem.miniTurretPointToBackdrop(correctedHeading);
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_DOWN);
        outtakeSubsystem.liftTo(liftTarget,outtakeSubsystem.liftPosition,1);
    }
}







