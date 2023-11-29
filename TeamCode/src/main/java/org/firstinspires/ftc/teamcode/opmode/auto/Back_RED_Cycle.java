package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.outoftheboxrobotics.photoncore.HAL.HAL;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    SampleMecanumDrive drive;
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
    Trajectory PreloadDrive1, PreloadDrive2, PreloadDrive3;


    // Define our start pose

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap); // road drive class
        SetAuto.setRedAuto();

        startPose = new Pose2d(StartPoseXBackdrop, StartPoseYBackdrop *S, StartPoseHeadingBackdrop *S+A);

        PreloadDrive1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d( PreloadPose1X,PreloadPose1Y*S, PreloadPose1Heading*S+A))
                .build();
        PreloadDrive2 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(PreloadPose2X,PreloadPose2Y*S, PreloadPose2Heading*S+A))
                .build();
        PreloadDrive3 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(PreloadPose3X,PreloadPose3Y*S,PreloadPose3Heading*S+A))
                .build();

        // initialize hardware
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } //

        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        cameraHardware.initWebcam(hardwareMap);


        // functions runs on start
        
        // Set inital pose
        drive.setPoseEstimate(startPose);

        // trajectories that aren't changing should all be here

        while (!isStarted()) { // initialization loop
            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
            telemetry.update();
        }


        waitForStart();
        if (isStopRequested()) return;


        // runs instantly once
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        autoTimer = GlobalTimer.milliseconds();

        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop
        
        currentState = AutoState.DELAY; // this go here?
        autoTimer = 0;
        numCycles = 0;
        
        cameraHardware.closeWebcam(); // reduces loop times

        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            outtakeSubsystem.outtakeReads(currentState == AutoState.DROP); // might need to change this
            intakeSubsystem.intakeReads(currentState == AutoState.GRAB_OFF_STACK);
            poseEstimate = drive.getPoseEstimate();

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

            drive.update();
            telemetry.update();
        }
        
    }
    
    public void autoSequence(){

        switch (currentState) {
            case DELAY:
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_UP);
                if (GlobalTimer.milliseconds() - autoTimer > 3000){
                    autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                    currentState = AutoState.PRELOAD_DRIVE;
                    if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.LEFT){
                        teamPropLocation = 1;
                        drive.followTrajectoryAsync(PreloadDrive1);
                        telemetry.addLine("left");
                    } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.CENTER){
                        teamPropLocation = 2;
                        drive.followTrajectoryAsync(PreloadDrive2);
                        telemetry.addLine("center");
                    } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.RIGHT){
                        teamPropLocation = 3;
                        drive.followTrajectoryAsync(PreloadDrive3);
                        telemetry.addLine("right");
                    }
                }
                break;
            case PRELOAD_DRIVE:
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_UP);
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
                             //   drive.followTrajectoryAsync();
                                currentState = AutoState.GRAB_OFF_STACK;
                            }
                        }
                    }
                }


                break;

            case GRAB_OFF_STACK:
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
                        if (intakeSubsystem.pixelsInIntake() || !drive.isBusy()){
                            currentState = AutoState.AFTER_GRAB_OFF_STACK;
                            intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.HOLDING);
                            autoTimer = GlobalTimer.milliseconds();
                          //  drive.followTrajectoryAsync();
                        }
                    }
                }
                break;

            case AFTER_GRAB_OFF_STACK:
                intakeSubsystem.intakeSlideInternalPID(-8,1);
                if (GlobalTimer.milliseconds() - autoTimer > 500){
                    intakeSubsystem.intakeSpin(0.5);
                    intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP);
                    if (intakeSubsystem.intakeSlidePosition < 10){
                        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.TRANSFER);
                        if (intakeSubsystem.intakeSlidePosition < 5 && intakeSubsystem.intakeChuteArmPosition < 140){
                            currentState = AutoState.OUTTAKE_PIXEL;
                            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                            autoTimer = GlobalTimer.milliseconds();
                        }
                    }
                } else {
                    intakeSubsystem.intakeSpin(-1);

                }
                break;

            case OUTTAKE_PIXEL:
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                outtakeSubsystem.liftTo(800,outtakeSubsystem.liftPosition,1);
                outtakeSubsystem.pitchToInternalPID(PITCH_LOW_DEGREE_TICKS,1);
                if (GlobalTimer.milliseconds() - autoTimer > 200){
               //     outtakeSubsystem.arm
                }
                break;
            case DROP:

                if (GlobalTimer.milliseconds() - autoTimer > 200){ // should go into a direct drive inwards
                    currentState = AutoState.PARK;
                    Trajectory Park = drive.trajectoryBuilder(poseEstimate)
                            .lineTo(new Vector2d(ParkMiddleX,ParkeMiddleY))
                            .build();
                    drive.followTrajectoryAsync(Park);
                    autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                }
                break;
            case PARK:

               if (!drive.isBusy()){
                    currentState = AutoState.IDLE; // doesn't have to drive anywhere, already in position hopefully
                }

                break;

            case IDLE:
                telemetry.addLine("WWWWWWWWWWW");
                //reset outtake servos
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





