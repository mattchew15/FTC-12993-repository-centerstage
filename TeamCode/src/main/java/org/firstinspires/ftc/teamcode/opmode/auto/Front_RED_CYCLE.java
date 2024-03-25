package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opmode.teleop.SimplicityDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.CameraHardware;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.*;

import android.provider.Settings;
import android.view.ViewTreeObserver;

@Autonomous(name = "Front Red Cycle Auto", group = "Autonomous")
public class Front_RED_CYCLE extends LinearOpMode {

    // class members
    ElapsedTime GlobalTimer;
    double autoTimer;

    int numCycles;
    double correctedHeading;
    int timesIntoStack;
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
        autoTrajectories = new AutoTrajectories(); // road drive class

        // initialize hardware

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } //



        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        //cameraHardware.initWebcam(hardwareMap);
        autoTrajectories.init(hardwareMap);
        // functions runs on start


        // trajectories that aren't changing should all be here
        while (!isStarted()) { // initialization loop
            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
            //outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
            outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition,telemetry);

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
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        autoTimer = GlobalTimer.milliseconds();

        goToPark = true;

        intakeSubsystem.intakeHardwareSetup();
        outtakeSubsystem.hardwareSetup();
        outtakeSubsystem.encodersReset();
        intakeSubsystem.intakeSlideMotorEncodersReset();
        outtakeSubsystem.cacheInitialPitchValue();

        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop
        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY); // don't touch this at all in auto
        autoTimer = 0;
        numCycles = 0;
        outtakeSubsystem.outtakeDistanceSensorValue = 100; // so that it doesn't do funky stuff
        autoTrajectories.drive.setPoseEstimate(autoTrajectories.startPoseFront);

        currentState = AutoState.DELAY;
        //cameraHardware.closeWebcam(); // reduces loop times

        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            outtakeSubsystem.outtakeReads(currentState == AutoState.OUTTAKE_PIXEL && xPosition > 24); // might need to change this
            intakeSubsystem.intakeReads(currentState == AutoState.GRAB_OFF_STACK || currentState == AutoState.AFTER_GRAB_OFF_STACK || currentState == AutoState.PLACE_AND_INTAKE);
            poseEstimate = autoTrajectories.drive.getPoseEstimate();

            // Print pose to telemetry

            loopTime.updateLoopTime(telemetry);
            telemetry.addData("Auto State", currentState);
            telemetry.addData("numCycles", numCycles);
            telemetry.addData("distance sensor value", outtakeSubsystem.outtakeDistanceSensorValue);
            telemetry.addData ("times into stack", timesIntoStack);
            //telemetry.addData("intakeMotorVelocity", intakeSubsystem.intakeVelocity);
            telemetry.addData("right Limit Switch", intakeSubsystem.rightArmLimitSwitchValue);
            telemetry.addData("left Limit Switch", intakeSubsystem.leftArmLimitSwitchValue);
            //telemetry.addData("s",S);
            telemetry.addLine("");
            telemetry.addData("liftPosition", ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition));
            telemetry.addData("pitch endcoder Position", outtakeSubsystem.pitchEncoderPosition);
            telemetry.addData("intakeSlidePosition", intakeSubsystem.intakeSlidePosition);
            telemetry.addData("lift Target Reached", outtakeSubsystem.liftTargetReached());
            autoSequence();
            if (currentState != AutoState.PRELOAD_DRIVE){
                outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition,telemetry);
            }


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
        if ((GlobalTimer.milliseconds() > 28500) && goToPark && currentState != AutoState.IDLE){
            goToPark = false;
            currentState = AutoState.PARK;
            autoTrajectories.park(poseEstimate,2);
        }

        switch (currentState) {
            case DELAY:
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.UPRIGHT);
                outtakeSubsystem.pitchToInternalPID(PITCH_PURPLE_PIXEL_POSITION,1);
                outtakeSubsystem.liftTo(5,outtakeSubsystem.liftPosition,1);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                outtakeSubsystem.miniTurretPointToBackdrop(correctedHeading);
                outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition,telemetry);
                if (GlobalTimer.milliseconds() - autoTimer > 0){
                    autoTimer = GlobalTimer.milliseconds();
                    currentState = AutoState.PRELOAD_DRIVE;
                    if (teamPropLocation == 1){
                        autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive1Front);
                        telemetry.addLine("left");
                    } else if (teamPropLocation == 2){
                        autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive2Front);
                        telemetry.addLine("center");
                    } else if (teamPropLocation == 3){

                        autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive3Front);
                        telemetry.addLine("right");
                    }
                }

                break;
            case PRELOAD_DRIVE:
                outtakeSubsystem.liftTo(2,outtakeSubsystem.liftPosition,1); // so rail doesn't hit
                intakeSubsystem.intakeSlideInternalPID(0,1);
                outtakeSubsystem.setOuttakePitchPurplePixelPosition();
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
                if (GlobalTimer.milliseconds() - autoTimer > 500){
                    outtakeSubsystem.outtakeRailState(S != 1? OuttakeSubsystem.OuttakeRailState.RIGHT: OuttakeSubsystem.OuttakeRailState.LEFT);
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_PURPLE);
                    // turret deposit logic
                    if (teamPropLocation == 1){
                        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.FRONT_PURPLE);
                    } else if (teamPropLocation == 2){
                        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                    } else if (teamPropLocation == 3){
                        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.BACK_PURPLE);
                    }

                    // pre spin intake or smth
                    if (GlobalTimer.milliseconds() - autoTimer > 2000){
                        intakeSubsystem.intakeSpin(1);
                    }
                }

                if (RED_AUTO? yPosition > -15*S : yPosition < -15*S){
                    if (teamPropLocation == 1){
                        intakeSubsystem.intakeSlideTo(0, intakeSubsystem.intakeSlidePosition,1);
                    } else if (teamPropLocation == 2){
                        intakeSubsystem.intakeSlideTo(170, intakeSubsystem.intakeSlidePosition,0.5);
                    } else if (teamPropLocation == 3){
                        intakeSubsystem.intakeSlideTo(220, intakeSubsystem.intakeSlidePosition,0.5);
                    }
                }

                if (!autoTrajectories.drive.isBusy()){
                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                    outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
                    currentState = AutoState.PLACE_AND_INTAKE;
                    autoTimer = GlobalTimer.milliseconds();

                }
                break;

            case PLACE_AND_INTAKE:
                intakeSubsystem.intakeSpin(1);
                if (teamPropLocation == 1){
                    intakeSubsystem.intakeSlideTo(50, intakeSubsystem.intakeSlidePosition,1);
                } else if (teamPropLocation == 2){
                    intakeSubsystem.intakeSlideTo(330, intakeSubsystem.intakeSlidePosition,1);
                } else if (teamPropLocation == 3){
                    intakeSubsystem.intakeSlideTo(350, intakeSubsystem.intakeSlidePosition,1);
                }
                if (intakeSubsystem.intakeSlideTargetReached() || GlobalTimer.milliseconds() - autoTimer > 600){
                    if (GlobalTimer.milliseconds() - autoTimer > 260 || intakeSubsystem.pixelsInIntake()){ // ensure pixels are in robot
                        intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
                        autoTrajectories.outtakeDriveMiddlePath(poseEstimate,16, 28, -31);
                        autoTimer = GlobalTimer.milliseconds();
                        currentState = AutoState.TRANSFER_PIXEL;
                    }
                } else {
                    autoTimer = GlobalTimer.milliseconds(); // spams timer reset - sneaky trick
                    intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
                }
                outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
                outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);

                if (outtakeSubsystem.pitchEncoderPosition > 23 && GlobalTimer.milliseconds() - autoTimer > 150){
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                    outtakeSubsystem.liftTo(0,outtakeSubsystem.liftPosition,1);
                    outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                } else if (GlobalTimer.milliseconds() - autoTimer > 70){
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.UPRIGHT);
                }
                break;

            case AFTER_PURPLE_DRIVE:
                /* // this is the trigger statement for this case in case we need it later
                currentState = AutoState.AFTER_PURPLE_DRIVE;
                        if (teamPropLocation == 1){
                            autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.AfterPreloadDrive1Front);
                        } else if (teamPropLocation == 2){
                            autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.AfterPreloadDrive2Front);
                        } else if (teamPropLocation == 3){
                            autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.AfterPreloadDrive3Front);
                        }
                 */


                /*
                intakeSubsystem.intakeSlideInternalPID(0,1);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                intakeSubsystem.intakeSpin(-1);
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP);
                if (RED_AUTO? yPosition < -26.5*S:yPosition > -26.5*S){
                   // autoTrajectories.outtakeDriveMiddlePath(poseEstimate, 20);
                    if (teamPropLocation == 2){
                        autoTrajectories.outtakeDriveMiddlePath(poseEstimate,16, 30, -31);
                    } else if (teamPropLocation == 1){
                        autoTrajectories.outtakeDriveTurnEndPath(poseEstimate,16, -173,32, 2);
                    } else if (teamPropLocation == 3){
                        autoTrajectories.outtakeDriveTurnEndPath(poseEstimate,16, 173,32, 2);

                    }

                    autoTimer = GlobalTimer.milliseconds();
                    currentState = AutoState.TRANSFER_PIXEL;
                }
                 */
                break;

            case TRANSFER_PIXEL:
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                liftDown(0.05);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                intakeSubsystem.intakeSlideInternalPID(-10,1);
                if (GlobalTimer.milliseconds() - autoTimer > 70){ // time for pixel holder to close
                    intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP);

                    if ((GlobalTimer.milliseconds() - autoTimer > 450) && timesIntoStack < 1 && numCycles != 0){ // go back into stack formore pixels case
                        if (!intakeSubsystem.pixelsInIntake()){
                            currentState = AutoState.GRAB_OFF_STACK;
                            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                            intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                            intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
                            timesIntoStack += 1;
                            autoTimer = GlobalTimer.milliseconds();
                            if (numCycles == 3){
                                autoTrajectories.goBackToStack(poseEstimate,7,3);
                            }
                            else {
                                autoTrajectories.goBackToStack(poseEstimate,7,2);
                            }
                        }
                    }

                    if (GlobalTimer.milliseconds() - autoTimer > 500){ // also time for pixel holder to close
                        intakeSubsystem.intakeSpin(0.5);
                        if (intakeSubsystem.intakeSlidePosition < 10 && ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) < 0.1){
                            intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.TRANSFER);
                            if (intakeSubsystem.intakeSlidePosition < 2){
                                currentState = AutoState.OUTTAKE_PIXEL;
                                autoTimer = GlobalTimer.milliseconds();
                            }
                        } else {

                        }
                    } else {
                        intakeSubsystem.intakeSpin(-1);
                    }
                } else {
                    intakeSubsystem.intakeSpin(0.9); // this doesn't happen for very long
                }
                break;

            case OUTTAKE_PIXEL: // faster transfer
                if (GlobalTimer.milliseconds() - autoTimer > 400 || (intakeSubsystem.chuteDetectorLimitSwitchValue && GlobalTimer.milliseconds() - autoTimer > 200)){
                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);

                    if (GlobalTimer.milliseconds() - autoTimer > 630){ // time for grippers to close
                        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);

                        if (numCycles == 0){ // for very first cycle
                            outtakeSubsystem.pitchToInternalPID(20,1);
                            //outtakeSubsystem.updateLiftTargetProfile(570);
                            outtakeSubsystem.liftTo(26,outtakeSubsystem.liftPosition,1);
                            railLogic();
                        } else if (numCycles == 1){
                            outtakeSubsystem.liftTo(25,outtakeSubsystem.liftPosition,1);
                            //outtakeSubsystem.updateLiftTargetProfile(650);
                            outtakeSubsystem.pitchToInternalPID(22,1);
                        } else if (numCycles == 2){
                            //outtakeSubsystem.updateLiftTargetProfile(734);
                            outtakeSubsystem.liftTo(27,outtakeSubsystem.liftPosition,1);
                            outtakeSubsystem.pitchToInternalPID(23,1);
                        } else if (numCycles == 3){
                            //outtakeSubsystem.updateLiftTargetProfile(780);
                            outtakeSubsystem.liftTo(28,outtakeSubsystem.liftPosition,1);
                            outtakeSubsystem.pitchToInternalPID(25,1);
                            outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.RIGHT);
                        }
                        //outtakeSubsystem.profileLiftCalculate();
                        if (numCycles != 2){
                            intakeSubsystem.intakeSlideTo(500, intakeSubsystem.intakeSlidePosition, 0.6);
                        } else {
                            intakeSubsystem.intakeSlideTo(0, intakeSubsystem.intakeSlidePosition, 1);
                        }

                        if (GlobalTimer.milliseconds() - autoTimer > 730){
                            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE); // slides go out before arm so transfer is good
                            if (GlobalTimer.milliseconds() - autoTimer > 920){ // once chute is down
                                outtakeSubsystem.miniTurretPointToBackdrop(correctedHeading);
                                pivotLogic();
                                intakeSubsystem.intakeSpin(0);

                                if (!autoTrajectories.drive.isBusy() || (outtakeSubsystem.outtakeDistanceSensorValue < 6.7 && numCycles != 0)) { // //  || outtakeSubsystem.outtakeDistanceSensorValue < 8.7
                                    // line above determines when we drop the pixels
                                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                                    outtakeSubsystem.outtakeDistanceSensorValue = 100; // so that it doesn't do funky stuff
                                    numCycles += 1;
                                    timesIntoStack = 0;
                                    autoTimer = GlobalTimer.milliseconds();

                                    if (numCycles == 1){
                                        autoTrajectories.driveIntoStackStraight(poseEstimate,8,2); // might cause an issue here
                                        currentState = AutoState.DROP;
                                    } else if (numCycles == 2){
                                        autoTrajectories.driveIntoStackStraight(poseEstimate,12,2); // might cause an issue here
                                        currentState = AutoState.DROP;
                                    } else if (numCycles == 3){
                                        autoTrajectories.driveIntoStackStageFromMiddlePathStraightEnd(poseEstimate,8, 30); // this is going to far lane
                                        currentState = AutoState.DROP;
                                    } else if (numCycles == 4){
                                        currentState = AutoState.PARK;
                                        autoTrajectories.park(poseEstimate,2);
                                    }
                                }
                            }
                        } else {
                            intakeSubsystem.intakeSpin(-0.7);
                        }
                    }
                }
                break;
            case DROP:
                // waits for robot to move away far enough - minimize this
                if (GlobalTimer.milliseconds() - autoTimer > 90){
                    outtakeSubsystem.liftToInternalPID(-0.6,1);
                    outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
                    if (GlobalTimer.milliseconds() - autoTimer > 110){ // not knock pixels off the backdrop
                        outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
                        //intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                        if (numCycles == 1){
                               intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP);
                        } else if (numCycles == 2){
                           intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE);
                        } else if (numCycles == 3){
                            // make intake arm height correct
                          intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP);
                        } else if (numCycles == 4) {
                         intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE);
                        }
                        currentState = AutoState.GRAB_OFF_STACK;
                        autoTimer = GlobalTimer.milliseconds();
                        intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);

                    }

                }

                break;

            // might need to make multiple of these states depending on the trajectory it follows -- again making methods would help alot with this
            case GRAB_OFF_STACK: // put each state in a method in antoher class and pass in timer variable

                if (numCycles == 1 || numCycles > 2){ // wait for the outtake rail to retract
                    if (GlobalTimer.milliseconds() - autoTimer > 200){
                        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                    }
                } else {
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                    outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                }

                outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS, 1);
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
                outtakeSubsystem.liftToInternalPID(0,1);
                outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);

                if (xPosition < 18){ //
                    intakeSubsystem.intakeSlideTo(1050, intakeSubsystem.intakeSlidePosition,1);
                    if (xPosition < 14) { // for some reason we can't extend the slides unless we are further in
                        intakeSubsystem.intakeSpin(1);
                        if (!autoTrajectories.drive.isBusy() || intakeSubsystem.leftArmLimitSwitchValue || intakeSubsystem.rightArmLimitSwitchValue){ // do stuff with sensor to make better
                            currentState = AutoState.AFTER_GRAB_OFF_STACK;
                            autoTimer = GlobalTimer.milliseconds();
                            if (numCycles > 2) {
                                autoTrajectories.outtakeDriveFromStraightTurnEndStageV2(poseEstimate,16, 177, 8);
                            } else {
                                autoTrajectories.outtakeDriveMiddlePath(poseEstimate,13, 27, MiddleLaneYDeposit);
                            }
                        }
                    } else{
                        if (numCycles == 3){
                            intakeSubsystem.intakeSpin(-1);
                        }
                    }
                } else if (numCycles != 3){
                    intakeSubsystem.intakeSlideTo(1050, intakeSubsystem.intakeSlidePosition,1);
                }
                if (intakeSubsystem.pixelsInIntake() && timesIntoStack !=0){ //this isn't true unless timesIntoStack has happened at least once
                    currentState = AutoState.AFTER_GRAB_OFF_STACK;
                    autoTimer = GlobalTimer.milliseconds();
                    if (numCycles > 2) {
                        autoTrajectories.outtakeDriveFromStraightTurnEndStageV2(poseEstimate,16, 175, 8);
                    } else {
                        autoTrajectories.outtakeDriveMiddlePath(poseEstimate,13, 27, MiddleLaneYDeposit);
                    }
                }
                break;

            // might need to make another state for the colour sensors similar to teleop
            case AFTER_GRAB_OFF_STACK:
                if (GlobalTimer.milliseconds() - autoTimer > 0){
                    /*
                    if (numCycles == 1 || numCycles == 3) {
                        //intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.FOUR);
                    }
                     */
                    if (numCycles == 2){
                        intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                    }
                    if (GlobalTimer.milliseconds() - autoTimer > 200){
                        autoTimer = GlobalTimer.milliseconds(); // resets timer
                        currentState = AutoState.TRANSFER_PIXEL;
                        intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
                    }
                }
                break;
            case PARK:
                if (GlobalTimer.milliseconds() - autoTimer > 200){
                   currentState = AutoState.IDLE; // doesn't have to drive anywhere, already in position hopefully
                    autoTimer = GlobalTimer.milliseconds(); // resets timer
                }
                break;

            case IDLE:
                telemetry.addLine("WWWWWWWWWWW");
                intakeSubsystem.intakeSlideInternalPID(-6,1);
                outtakeSubsystem.liftToInternalPID(-8,1);
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
                outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS, 1);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                if (GlobalTimer.milliseconds() - autoTimer > 400){
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                }
                outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
                break;





        }
    }
    public void pivotLogic (){
        if (numCycles == 0){
            if (teamPropLocation == 1){
                if (BLUE_AUTO){
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
                }
                else {
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
                }
            } else if (teamPropLocation == 2){
                if (RED_AUTO){
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
                }
                else {
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
                }
            } else if (teamPropLocation == 3){
                if (RED_AUTO){
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
                }
                else {
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
                }
            }
        } else {
            if (BLUE_AUTO){
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
            }
            else {
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
            }
        }

    }
    public void railLogic(){
        if (teamPropLocation == 2){
            outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
        } else if (teamPropLocation == 1){
            outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.LEFT);
        } else if (teamPropLocation == 3){
            outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.RIGHT);
        }
    }

    public void liftDown(double liftTheshold){
        if (ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) < liftTheshold){ // if the lift is where we want it
            outtakeSubsystem.liftToInternalPID(-0.15,1);
        } else {
            outtakeSubsystem.liftToInternalPID(-3,1);
        }
    }
}


