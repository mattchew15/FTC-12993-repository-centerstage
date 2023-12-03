/*
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

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoGlobals.*;
import static org.firstinspires.ftc.teamcode.system.vision.YCrCbRedTeamPropDetectorPipeline.RED_POSITION;

import android.provider.Settings;
import android.view.ViewTreeObserver;

@Autonomous(name = "Back Red Cycle Auto", group = "Autonomous")
public class Back_RED_Cycle extends LinearOpMode {

    // class members
    ElapsedTime GlobalTimer;
    double autoTimer;

    int numCycles;
    int teamPropLocation;
    int liftTarget;

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
    Pose2d startPose;
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


        // functions runs on start
        
        // Set inital pose
        autoTrajectories.drive.setPoseEstimate(autoTrajectories.startPoseBack);

        // trajectories that aren't changing should all be here

        while (!isStarted()) { // initialization loop
            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
            telemetry.update();
        }


        waitForStart();
        if (isStopRequested()) return;


        // runs instantly once
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        autoTimer = GlobalTimer.milliseconds();

        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop
        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY); // don't touch this at all in auto


        currentState = AutoState.DELAY; // this go here?
        autoTimer = 0;
        numCycles = 0;
        
        cameraHardware.closeWebcam(); // reduces loop times

        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            outtakeSubsystem.outtakeReads(currentState == AutoState.OUTTAKE_PIXEL); // might need to change this
            intakeSubsystem.intakeReads(currentState == AutoState.GRAB_OFF_STACK || currentState == AutoState.AFTER_GRAB_OFF_STACK);
            poseEstimate = autoTrajectories.drive.getPoseEstimate();

            // Print pose to telemetry
            loopTime.updateLoopTime(telemetry);
            telemetry.addData("Auto State", currentState);
            telemetry.addLine("");
            telemetry.addData("liftPosition", outtakeSubsystem.liftPosition);
            telemetry.addData("pitchPosition", outtakeSubsystem.pitchPosition);
            telemetry.addData("intakeSlidePosition", intakeSubsystem.intakeSlidePosition);

            autoSequence();


            //xPosition = poseEstimate.getX();              // could put the x,y and heading into globals
            //yPosition = poseEstimate.getY();
            //headingPosition = poseEstimate.getHeading();

            autoTrajectories.drive.update();
            telemetry.update();
        }
        
    }
    
    public void autoSequence(){

        switch (currentState) {
            case DELAY:
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.UPRIGHT);
                if (GlobalTimer.milliseconds() - autoTimer > 3000){
                    autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                    currentState = AutoState.PRELOAD_DRIVE;
                    if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.LEFT){
                        teamPropLocation = 1;
                        autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive1);
                        telemetry.addLine("left");
                    } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.CENTER){
                        teamPropLocation = 2;
                        autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive2);
                        telemetry.addLine("center");
                    } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.RIGHT){
                        teamPropLocation = 3;
                        autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive3);
                        telemetry.addLine("right");
                    }
                }
                break;
            case PRELOAD_DRIVE:
                if (GlobalTimer.milliseconds() - autoTimer > 300) {
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_UP);
                }
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                if (poseEstimate.getY() > -25){
                    autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                    currentState = AutoState.OUT_AFTER_PRELOAD_DRIVE;
                }
                break;

            case OUT_AFTER_PRELOAD_DRIVE:
                if (teamPropLocation == 1){
                    intakeSubsystem.intakeSlideTo(100, intakeSubsystem.intakeSlidePosition,1);
                    liftTarget = 250;
                    outtakePreload(poseEstimate);
                } else if (teamPropLocation == 2){
                    intakeSubsystem.intakeSlideTo(200, intakeSubsystem.intakeSlidePosition,1);
                    liftTarget = 350;
                    outtakePreload(poseEstimate);
                } else if (teamPropLocation == 3){
                    intakeSubsystem.intakeSlideTo(300, intakeSubsystem.intakeSlidePosition,1);
                    liftTarget = 450;
                    outtakePreload(poseEstimate);
                }
                if (intakeSubsystem.intakeSlideTargetReached() && outtakeSubsystem.liftTargetReached()){
                    currentState = AutoState.DROP_AFTER_PRELOAD;
                }
                break;

            case DROP_AFTER_PRELOAD:
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP);
                if (GlobalTimer.milliseconds() - autoTimer > 200){
                    intakeSubsystem.intakeSlideInternalPID(100,1); // retract the slides
                    if (GlobalTimer.milliseconds() - autoTimer > 500){ // this can be improved by adding another state?
                        outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                        if (GlobalTimer.milliseconds() - autoTimer > 750){
                            outtakeSubsystem.liftTo(100, outtakeSubsystem.liftPosition, 1);
                            if (GlobalTimer.milliseconds() - autoTimer > 900){
                                autoTimer = GlobalTimer.milliseconds();
                                if (teamPropLocation == 1){
                                    autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.driveIntoStackAfterBackPreload1);
                                } else if (teamPropLocation == 2){
                                    autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.driveIntoStackAfterBackPreload2);
                                } else if (teamPropLocation == 3){
                                    autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.driveIntoStackAfterBackPreload3);
                                }
                                currentState = AutoState.GRAB_OFF_STACK;
                            }
                        }
                    }
                }


                break;

            // might need to make multiple of these states depending on the trajectory it follows -- again making methods would help alot with this
            case GRAB_OFF_STACK: // put each state in a method in antoher class and pass in timer variable

                outtakeSubsystem.pitchToInternalPID(SIXTY_DEGREE_TICKS, 1);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                outtakeSubsystem.liftToInternalPID(0,1);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);

                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP);
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.OPEN);

                if (poseEstimate.getX() < -55){
                    intakeSubsystem.intakeSlideTo(680, intakeSubsystem.intakeSlidePosition,1);
                    if (poseEstimate.getX() < -45) {
                        intakeSubsystem.intakeSpin(1);
                        if (intakeSubsystem.backColourSensorValue > 2500 || !autoTrajectories.drive.isBusy()){ // do stuff with sensor to make better
                            currentState = AutoState.AFTER_GRAB_OFF_STACK;
                            autoTimer = GlobalTimer.milliseconds();
                            autoTrajectories.outtakeDriveMiddlePath(poseEstimate,20, 26.5, MiddleLaneY);
                        }
                    }
                }
                break;

                // might need to make another state for the colour sensors similar to teleop
            case AFTER_GRAB_OFF_STACK:
                intakeSubsystem.intakeSpin(1);
                if (GlobalTimer.milliseconds() - autoTimer > 200){
                    if (intakeSubsystem.pixelsInIntake() || GlobalTimer.milliseconds() - autoTimer > 500) {
                        autoTimer = GlobalTimer.milliseconds(); // resets timer
                        currentState = AutoState.TRANSFER_PIXEL;
                        intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.HOLDING);
                    }
                }

            case TRANSFER_PIXEL:
                intakeSubsystem.intakeSlideInternalPID(-8,1);
                if (GlobalTimer.milliseconds() - autoTimer > 100){ // time for pixel holder to close
                    if (GlobalTimer.milliseconds() - autoTimer > 500){
                        intakeSubsystem.intakeSpin(0.5);
                        if (intakeSubsystem.intakeSlidePosition < 10){
                            intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.TRANSFER);
                            if (intakeSubsystem.intakeSlidePosition < 5 && intakeSubsystem.intakeChuteArmPosition < 160){
                                currentState = AutoState.OUTTAKE_PIXEL;
                                autoTimer = GlobalTimer.milliseconds();
                            }
                        } else {
                            intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP);
                        }
                    } else {
                        intakeSubsystem.intakeSpin(-1);

                    }
                } else {
                    intakeSubsystem.intakeSpin(1); // this doesn't happen for very long
                }

                break;

            case OUTTAKE_PIXEL:
                if (GlobalTimer.milliseconds() - autoTimer > 200){ // wait for chute to go in properly
                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                    if (GlobalTimer.milliseconds() - autoTimer > 330){ // time for grippers to close
                        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                        outtakeSubsystem.liftTo(700,outtakeSubsystem.liftPosition,1);
                        outtakeSubsystem.pitchToInternalPID(PITCH_LOW_DEGREE_TICKS,1);
                        if (GlobalTimer.milliseconds() - autoTimer > 400){
                            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_DOWN); // slides go out before arm so transfer is good
                            if (GlobalTimer.milliseconds() - autoTimer > 450 || intakeSubsystem.intakeChuteArmPosition > 200){ // once chute is down
                                intakeSubsystem.intakeSlideTo(250, intakeSubsystem.intakeSlidePosition, 1); // line above may limit speed of drop
                                if ((outtakeSubsystem.outtakeDistanceSensorValue < 5 || !autoTrajectories.drive.isBusy()) && outtakeSubsystem.liftTargetReached()) { // || poseEstimate.getX() > 35 this could make it faster on the first cycle
                                    // line above determines when we drop the pixels
                                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                                    numCycles += 1;
                                    autoTimer = GlobalTimer.milliseconds();

                                    if (numCycles == 1){
                                        autoTrajectories.driveIntoStackStraight(poseEstimate,20,2);
                                        currentState = AutoState.DROP;
                                    } else if (numCycles == 2){
                                        autoTrajectories.driveIntoStackStageFromMiddlePathStraightEnd(poseEstimate,20);
                                        currentState = AutoState.DROP;
                                    } else if (numCycles == 3){
                                        autoTrajectories.park(poseEstimate,2);
                                        currentState = AutoState.PARK;
                                    }

                                }
                            }
                        }

                    }
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
                       if (numCycles == 1){
                           intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE);
                       } else if (numCycles == 2){
                           intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                       } else if (numCycles == 4){
                           // make intake arm height correct
                       }
                       currentState = AutoState.GRAB_OFF_STACK;
                       autoTimer = GlobalTimer.milliseconds();

                   }

                }
                break;
            case PARK:

               if (!autoTrajectories.drive.isBusy()){
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
        outtakeSubsystem.miniTurretPointToBackdrop(Math.toDegrees(poseEstimate.getHeading()));
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_DOWN);
        outtakeSubsystem.pitchToInternalPID(PITCH_LOW_DEGREE_TICKS,1);
        outtakeSubsystem.liftTo(liftTarget,outtakeSubsystem.liftPosition,1);
    }
}


 */




