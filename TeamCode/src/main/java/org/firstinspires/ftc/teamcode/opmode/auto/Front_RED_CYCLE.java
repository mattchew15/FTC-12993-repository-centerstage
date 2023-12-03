package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.teleop.SimplicityDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
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

@Autonomous(name = "Front Red Cycle Auto", group = "Autonomous")
public class Front_RED_CYCLE extends LinearOpMode {

    // class members
    ElapsedTime GlobalTimer;
    double autoTimer;

    int numCycles;
    int teamPropLocation;
    int liftTarget;
    double correctedHeading;

    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    CameraHardware cameraHardware = new CameraHardware();
    AutoTrajectories autoTrajectories;
    //Accessories
    LoopTime loopTime = new LoopTime();


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
        autoTrajectories.drive.setPoseEstimate(autoTrajectories.startPoseFront);

        // trajectories that aren't changing should all be here

        while (!isStarted()) { // initialization loop
            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP);
            if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.LEFT){
                telemetry.addLine("left");
            } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.CENTER){
                telemetry.addLine("center");
            } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.RIGHT){
                telemetry.addLine("right");
            }
            telemetry.update();
        }


        waitForStart();
        if (isStopRequested()) return;


        // runs instantly once
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        autoTimer = GlobalTimer.milliseconds();

        intakeSubsystem.intakeHardwareSetup();
        outtakeSubsystem.hardwareSetup();
        outtakeSubsystem.encodersReset();
        intakeSubsystem.intakeSlideMotorEncodersReset();

        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop
        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY); // don't touch this at all in auto


        currentState = AutoState.DELAY;
        autoTimer = 0;
        numCycles = 0;

        cameraHardware.closeWebcam(); // reduces loop times

        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            outtakeSubsystem.outtakeReads(false); // might need to change this
            intakeSubsystem.intakeReads(currentState == AutoState.GRAB_OFF_STACK || currentState == AutoState.AFTER_GRAB_OFF_STACK || currentState == AutoState.PLACE_AND_INTAKE);
            poseEstimate = autoTrajectories.drive.getPoseEstimate();

            // Print pose to telemetry
            loopTime.updateLoopTime(telemetry);
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
            correctedHeading = angleWrap(Math.toDegrees(headingPosition));


            autoTrajectories.drive.update();
            telemetry.update();
        }

    }

    public void autoSequence(){
        switch (currentState) {
            case DELAY:
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.UPRIGHT);
                outtakeSubsystem.pitchToInternalPID(680,1);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                if (GlobalTimer.milliseconds() - autoTimer > 0){
                    autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                    currentState = AutoState.PRELOAD_DRIVE;
                    if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.LEFT){
                        teamPropLocation = 1;
                        autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive1Front);
                        telemetry.addLine("left");
                    } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.CENTER){
                        teamPropLocation = 2;
                        autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive2Front);
                        telemetry.addLine("center");
                    } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.RIGHT){
                        teamPropLocation = 3;
                        autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive3Front);
                        telemetry.addLine("right");
                    }
                }
                break;
            case PRELOAD_DRIVE:
                intakeSubsystem.intakeSlideInternalPID(0,1);
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop

                if (GlobalTimer.milliseconds() - autoTimer > 1000){
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_UP);
                }
                if (!autoTrajectories.drive.isBusy()){
                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);

                    outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                    currentState = AutoState.PLACE_AND_INTAKE;
                    autoTimer = GlobalTimer.milliseconds();
                    intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
                }
                break;

            case PLACE_AND_INTAKE:
                intakeSubsystem.intakeSpin(1);
                if (teamPropLocation == 1){
                    intakeSubsystem.intakeSlideTo(100, intakeSubsystem.intakeSlidePosition,1);
                } else if (teamPropLocation == 2){
                    intakeSubsystem.intakeSlideTo(295, intakeSubsystem.intakeSlidePosition,1);
                } else if (teamPropLocation == 3){
                    intakeSubsystem.intakeSlideTo(480, intakeSubsystem.intakeSlidePosition,1);

                }
                if (intakeSubsystem.intakeSlideTargetReached()){
                    if (GlobalTimer.milliseconds() - autoTimer > 700){ // ensure pixels are in robot
                        intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.HOLDING);
                        currentState = AutoState.AFTER_PURPLE_DRIVE;
                        if (teamPropLocation == 1){
                            autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.AfterPreloadDrive1Front);
                        } else if (teamPropLocation == 2){
                            autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.AfterPreloadDrive2Front);
                        } else if (teamPropLocation == 3){
                            autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.AfterPreloadDrive3Front);
                        }
                    }
                } else {
                    autoTimer = GlobalTimer.milliseconds(); // spams timer reset - sneaky trick
                    intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.OPEN);
                }
                outtakeSubsystem.pitchToInternalPID(SIXTY_DEGREE_TICKS,1);
                if (GlobalTimer.milliseconds() - autoTimer > 150){
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                }
                break;

            case AFTER_PURPLE_DRIVE:
                intakeSubsystem.intakeSlideInternalPID(0,1);
                intakeSubsystem.intakeSpin(-1);
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP);
                if (yPosition < -27.5){
                   // autoTrajectories.outtakeDriveMiddlePath(poseEstimate, 20);
                    autoTrajectories.outtakeDriveMiddlePath(poseEstimate,20, 30, -30);
                    autoTimer = GlobalTimer.milliseconds();
                    currentState = AutoState.TRANSFER_PIXEL;
                }
                break;

            case TRANSFER_PIXEL:

                intakeSubsystem.intakeSlideInternalPID(-3,1);
                if (GlobalTimer.milliseconds() - autoTimer > 170){ // time for pixel holder to close
                    if (GlobalTimer.milliseconds() - autoTimer > 350){
                        intakeSubsystem.intakeSpin(0.5);
                        if (intakeSubsystem.intakeSlidePosition < 5){
                            intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.TRANSFER);
                            if (intakeSubsystem.intakeSlidePosition < 2){
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
                    intakeSubsystem.intakeSpin(0.5); // this doesn't happen for very long
                }


                break;

            case OUTTAKE_PIXEL:
                if (GlobalTimer.milliseconds() - autoTimer > 550 ){ // wait for chute to go in properly
                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                    if (GlobalTimer.milliseconds() - autoTimer > 650){ // time for grippers to close
                        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);

                        if (numCycles == 0){
                            outtakeSubsystem.pitchToInternalPID(1220,1);
                            outtakeSubsystem.updateLiftTargetProfile(570);
                            //outtakeSubsystem.liftTo(699,outtakeSubsystem.liftPosition,1);
                        } else if (numCycles == 1){
                            //outtakeSubsystem.liftTo(725,outtakeSubsystem.liftPosition,1);
                            outtakeSubsystem.updateLiftTargetProfile(690);
                            outtakeSubsystem.pitchToInternalPID(980,1);
                        } else if (numCycles == 2){
                            outtakeSubsystem.updateLiftTargetProfile(729);
                            //outtakeSubsystem.liftTo(738,outtakeSubsystem.liftPosition,1);
                            outtakeSubsystem.pitchToInternalPID(840,1);
                        } else if (numCycles == 3){
                            outtakeSubsystem.updateLiftTargetProfile(738);
                            //outtakeSubsystem.liftTo(738,outtakeSubsystem.liftPosition,1);
                            outtakeSubsystem.pitchToInternalPID(900,1);
                        }
                        outtakeSubsystem.profileLiftCalculate();
                        if (GlobalTimer.milliseconds() - autoTimer > 950){
                            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_DOWN); // slides go out before arm so transfer is good
                            if (GlobalTimer.milliseconds() - autoTimer > 1150){ // once chute is down
                                outtakeSubsystem.miniTurretPointToBackdrop(correctedHeading);
                                if (numCycles == 0){
                                    if (teamPropLocation == 1){
                                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
                                    } else if (teamPropLocation == 2){
                                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
                                    } else if (teamPropLocation == 3){
                                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
                                    }
                                } else {
                                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
                                }
                                if (numCycles < 2){
                                    intakeSubsystem.intakeSlideTo(250, intakeSubsystem.intakeSlidePosition, 1); // line above may limit speed of drop
                                } else {
                                    intakeSubsystem.intakeSlideTo(0, intakeSubsystem.intakeSlidePosition, 1); // line above may limit speed of drop
                                }
                                telemetry.addLine("DRIVE IS SITLL BUSY!!!!!");
                                if (!autoTrajectories.drive.isBusy()) { // // || xPosition > 35 this could make it faster on the first cycle
                                    // line above determines when we drop the pixels
                                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                                    numCycles += 1;
                                    autoTimer = GlobalTimer.milliseconds();

                                    if (numCycles == 1){
                                        autoTrajectories.driveIntoStackStraight(poseEstimate,12,2); // might cause an issue here
                                        //  autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.driveIntoStackStraightMIDDLE); // not a live trajectory for this one
                                        currentState = AutoState.DROP;
                                    } else if (numCycles == 2){
                                        autoTrajectories.driveIntoStackStraight(poseEstimate,18,2); // might cause an issue here
                                        currentState = AutoState.DROP;
                                    } else if (numCycles == 3){
                                        autoTrajectories.driveIntoStackStageFromMiddlePathStraightEnd(poseEstimate,20);
                                        currentState = AutoState.DROP;
                                    } else if (numCycles == 4){
                                        currentState = AutoState.PARK;
                                        autoTrajectories.park(poseEstimate,2);
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
                            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
                        } else if (numCycles == 2){
                            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE);
                        } else if (numCycles == 3){
                            // make intake arm height correct
                            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
                        } else if (numCycles == 4) {
                            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE);
                        }
                        currentState = AutoState.GRAB_OFF_STACK;
                        autoTimer = GlobalTimer.milliseconds();
                        intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.OPEN);

                    }

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


                if (xPosition < 0){ //
                    intakeSubsystem.intakeSlideTo(700, intakeSubsystem.intakeSlidePosition,1);
                    if (xPosition < -12) {
                        intakeSubsystem.intakeSpin(1);
                        if (xPosition < -35.2){ // do stuff with sensor to make better

                                currentState = AutoState.AFTER_GRAB_OFF_STACK;
                                autoTimer = GlobalTimer.milliseconds();
                                if (numCycles > 2) {
                                   autoTrajectories.outtakeDriveTurnEndPath(poseEstimate,22,150,3);
                                } else {
                                    autoTrajectories.outtakeDriveMiddlePath(poseEstimate,22, 25.5, MiddleLaneY);
                                }
                        }
                    }
                }
                break;

            // might need to make another state for the colour sensors similar to teleop
            case AFTER_GRAB_OFF_STACK:
                /*
                if (intakeSubsystem.pixelsInIntake()){
                    intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.HOLDING);
                }
                 */
                if (GlobalTimer.milliseconds() - autoTimer > 100){
                    if (numCycles == 1 || numCycles == 3) {
                        intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.FOUR);
                    } else if (numCycles == 2){
                        intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                    }
                    if (GlobalTimer.milliseconds() - autoTimer > 300){
                        autoTimer = GlobalTimer.milliseconds(); // resets timer
                        currentState = AutoState.TRANSFER_PIXEL;
                        intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.HOLDING);
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
}





