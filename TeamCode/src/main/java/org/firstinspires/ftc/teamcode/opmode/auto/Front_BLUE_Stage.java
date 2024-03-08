
/*package org.firstinspires.ftc.teamcode.opmode.auto;
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

@Autonomous(name = "Front Blue Stage Auto", group = "Autonomous")
public class Front_BLUE_Stage extends LinearOpMode {

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

        SetAuto.setBlueAuto();
        autoTrajectories = new AutoTrajectories(); // road drive class

        // initialize hardware
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } //

        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        cameraHardware.initWebcam(hardwareMap);
        autoTrajectories.init(hardwareMap);


        // functions runs on start


        // trajectories that aren't changing should all be here

        while (!isStarted()) { // initialization loop
            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
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

        intakeSubsystem.intakeHardwareSetup();
        outtakeSubsystem.hardwareSetup();
        outtakeSubsystem.encodersReset();
        intakeSubsystem.intakeSlideMotorEncodersReset();


        goToPark = true;

        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop
        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY); // don't touch this at all in auto
        autoTimer = 0;
        numCycles = 0;
        outtakeSubsystem.outtakeDistanceSensorValue = 100; // so that it doesn't do funky stuff
        autoTrajectories.drive.setPoseEstimate(autoTrajectories.startPoseFront);

        currentState = AutoState.DELAY;
        cameraHardware.closeWebcam(); // reduces loop times

        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            outtakeSubsystem.outtakeReads(currentState == AutoState.OUTTAKE_PIXEL && xPosition > 24); // might need to change this
            intakeSubsystem.intakeReads(currentState == AutoState.GRAB_OFF_STACK || currentState == AutoState.AFTER_GRAB_OFF_STACK || currentState == AutoState.PLACE_AND_INTAKE);
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
                outtakeSubsystem.pitchToInternalPID(570,1);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                outtakeSubsystem.miniTurretPointToBackdrop(correctedHeading);
                if (GlobalTimer.milliseconds() - autoTimer > 0){
                    autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
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
                    telemetry.addData("S", S);
                }

                break;
            case PRELOAD_DRIVE:
                intakeSubsystem.intakeSlideInternalPID(0,1);
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
                if (GlobalTimer.milliseconds() - autoTimer > 900){
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_PURPLE);
                    if (GlobalTimer.milliseconds() - autoTimer > 1500){
                        intakeSubsystem.intakeSpin(1);
                    }
                }

                if (RED_AUTO? yPosition > -15*S : yPosition < -15*S){
                    if (teamPropLocation == 1){
                        intakeSubsystem.intakeSlideTo(0, intakeSubsystem.intakeSlidePosition,1);
                    } else if (teamPropLocation == 2){
                        intakeSubsystem.intakeSlideTo(200, intakeSubsystem.intakeSlidePosition,0.5);
                    } else if (teamPropLocation == 3){
                        intakeSubsystem.intakeSlideTo(350, intakeSubsystem.intakeSlidePosition,0.5);
                    }
                }

                if (!autoTrajectories.drive.isBusy()){
                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);

                    outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                    currentState = AutoState.PLACE_AND_INTAKE;
                    autoTimer = GlobalTimer.milliseconds();

                }
                break;

            case PLACE_AND_INTAKE:
                intakeSubsystem.intakeSpin(1);
                if (teamPropLocation == 1){
                    intakeSubsystem.intakeSlideTo(0, intakeSubsystem.intakeSlidePosition,1);
                } else if (teamPropLocation == 2){
                    intakeSubsystem.intakeSlideTo(285, intakeSubsystem.intakeSlidePosition,1);
                } else if (teamPropLocation == 3){
                    intakeSubsystem.intakeSlideTo(500, intakeSubsystem.intakeSlidePosition,1);

                }
                if (intakeSubsystem.intakeSlideTargetReached() || GlobalTimer.milliseconds() - autoTimer > 500){
                    if (GlobalTimer.milliseconds() - autoTimer > 500){ // ensure pixels are in robot
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
                if (GlobalTimer.milliseconds() - autoTimer > 100){
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                }
                break;

            case AFTER_PURPLE_DRIVE:
                intakeSubsystem.intakeSlideInternalPID(0,1);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                intakeSubsystem.intakeSpin(-1);
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP);
                if (RED_AUTO? yPosition < -26.5*S:yPosition > -26.5*S){
                    // autoTrajectories.outtakeDriveMiddlePath(poseEstimate, 20);


                    if (teamPropLocation == 2){
                        autoTrajectories.outtakeDriveMiddlePath(poseEstimate,16, 30, -31);
                    } else if (teamPropLocation == 1){
                        autoTrajectories.outtakeDriveTurnEndPath(poseEstimate,16, RED_AUTO?TurnLeftFrontPreload:TurnRightFrontPreload,32, 2);
                    } else if (teamPropLocation == 3){
                        autoTrajectories.outtakeDriveTurnEndPath(poseEstimate,16, RED_AUTO?TurnRightFrontPreload:TurnLeftFrontPreload,32, 2);

                    }

                    autoTimer = GlobalTimer.milliseconds();
                    currentState = AutoState.TRANSFER_PIXEL;
                }
                break;

            case TRANSFER_PIXEL:
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                intakeSubsystem.intakeSlideInternalPID(-6,1);
                if (GlobalTimer.milliseconds() - autoTimer > 100){ // time for pixel holder to close
                    intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP);


                    // if it doesn't detect pixels it will go drive into stack state
                    if ((GlobalTimer.milliseconds() - autoTimer > 600) && timesIntoStack < 3 && numCycles != 0){
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
                        intakeSubsystem.intakeSpin(0.5);

                        if (intakeSubsystem.intakeSlidePosition < 5){
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

            case OUTTAKE_PIXEL:
                if (numCycles == 0? GlobalTimer.milliseconds() - autoTimer > 400: GlobalTimer.milliseconds() - autoTimer > 500 ){ // wait for chute to go in properly
                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                    /*
                    if(numCycles == 0 && GlobalTimer.milliseconds() - autoTimer > 530){
                        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                    }

                    if (GlobalTimer.milliseconds() - autoTimer > 630){ // time for grippers to close
                        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);

                        if (numCycles == 0){
                            outtakeSubsystem.pitchToInternalPID(1200,1);
                            outtakeSubsystem.updateLiftTargetProfile(570);
                            //outtakeSubsystem.liftTo(699,outtakeSubsystem.liftPosition,1);
                        } else if (numCycles == 1){
                            //outtakeSubsystem.liftTo(725,outtakeSubsystem.liftPosition,1);
                            outtakeSubsystem.updateLiftTargetProfile(650);
                            outtakeSubsystem.pitchToInternalPID(980,1);
                        } else if (numCycles == 2){
                            outtakeSubsystem.updateLiftTargetProfile(734);
                            //outtakeSubsystem.liftTo(738,outtakeSubsystem.liftPosition,1);
                            outtakeSubsystem.pitchToInternalPID(850,1);
                        } else if (numCycles == 3){
                            outtakeSubsystem.updateLiftTargetProfile(780);
                            //outtakeSubsystem.liftTo(738,outtakeSubsystem.liftPosition,1);
                            outtakeSubsystem.pitchToInternalPID(900,1);
                        }
                        outtakeSubsystem.profileLiftCalculate();
                        if (GlobalTimer.milliseconds() - autoTimer > 730){
                            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_DOWN); // slides go out before arm so transfer is good
                            if (GlobalTimer.milliseconds() - autoTimer > 920){ // once chute is down
                                outtakeSubsystem.miniTurretPointToBackdrop(correctedHeading);
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

                                intakeSubsystem.intakeSlideTo(0, intakeSubsystem.intakeSlidePosition, 1); // line above may limit speed of drop

                                telemetry.addLine("DRIVE IS SITLL BUSY!!!!!");
                                if (!autoTrajectories.drive.isBusy() || (outtakeSubsystem.outtakeDistanceSensorValue < 6.7 && numCycles != 0)) { // //  || outtakeSubsystem.outtakeDistanceSensorValue < 8.7
                                    // line above determines when we drop the pixels
                                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                                    outtakeSubsystem.outtakeDistanceSensorValue = 100; // so that it doesn't do funky stuff
                                    numCycles += 1;
                                    timesIntoStack = 0;
                                    autoTimer = GlobalTimer.milliseconds();

                                    if (numCycles == 1){
                                        autoTrajectories.driveIntoStackStageFromMiddlePathStraightEnd(poseEstimate,8, 30);
                                        currentState = AutoState.DROP;
                                    } else if (numCycles == 2){
                                        autoTrajectories.driveIntoStackStraightAfterAngledOuttakeSTAGE(poseEstimate, 16);
                                        currentState = AutoState.DROP;
                                    } else if (numCycles == 3){
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
                        //     intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
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

                if (intakeSubsystem.robotVoltage < 6.5){
                    currentState = AutoState.PARK;
                    autoTrajectories.park(poseEstimate,2);
                }

                if (xPosition < 6){ //
                    intakeSubsystem.intakeSlideTo(700, intakeSubsystem.intakeSlidePosition,1);
                    if (xPosition < -12) {

                        intakeSubsystem.intakeSpin(1);
                        if (!autoTrajectories.drive.isBusy()){ // do stuff with sensor to make better

                            currentState = AutoState.AFTER_GRAB_OFF_STACK;
                            autoTimer = GlobalTimer.milliseconds();
                            autoTrajectories.outtakeDriveFromStraightTurnEndStageV2(poseEstimate,16, 155,8);
                        }
                    } else{
                        if (numCycles == 1){
                            intakeSubsystem.intakeSpin(-1);
                        }
                    }
                }
                if (intakeSubsystem.pixelsInIntake() && timesIntoStack !=0){
                    currentState = AutoState.AFTER_GRAB_OFF_STACK;
                    autoTimer = GlobalTimer.milliseconds();
                    autoTrajectories.outtakeDriveFromStraightTurnEndStageV2(poseEstimate,16, 155,8);
                }
                break;

            // might need to make another state for the colour sensors similar to teleop
            case AFTER_GRAB_OFF_STACK:
                if (GlobalTimer.milliseconds() - autoTimer > 0){
                    if (numCycles == 1 || numCycles == 3) {
                        //   intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.FOUR);
                    } else if (numCycles == 2){
                        intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                    }
                    if (GlobalTimer.milliseconds() - autoTimer > 200){
                        autoTimer = GlobalTimer.milliseconds(); // resets timer
                        currentState = AutoState.TRANSFER_PIXEL;
                        intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.HOLDING);
                    }
                }
                break;
            case PARK:
                if (GlobalTimer.milliseconds() - autoTimer > 200){
                    currentState = AutoState.IDLE; // doesn't have to drive anywhere, already in position hopefully
                    autoTrajectories.park(poseEstimate,2);
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




 */


