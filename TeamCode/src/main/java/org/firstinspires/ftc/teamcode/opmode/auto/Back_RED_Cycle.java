
package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

@Autonomous(name = "Back Red Cycle Auto", group = "Autonomous")
public class Back_RED_Cycle extends LinearOpMode {

    // class members
    ElapsedTime GlobalTimer;
    double autoTimer;

    int numCycles;

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
        OUTTAKE_CONE_AFTER_PRELOAD,
        OUTTAKE_CONE,
        DROP,
        GRAB_OFF_STACK,
        AFTER_GRAB_OFF_STACK,
        TRANSFER_CONE,
        OUTTAKE_CONE_NO_INTAKE_SLIDES,
        RETRACT_SLIDES,
        DRIVE_OTHER_SIDE_AND_TRANSFER,
        TURN_OTHER_STACK,
        PARK,
        IDLE,
    }


    AutoState currentState;

    Pose2d startPose;

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


        // out cone stack position


        // functions runs on start
        
        // Set inital pose
        drive.setPoseEstimate(startPose);

        // trajectories that aren't changing should all be here

        while (!isStarted()) { // initialization loop
            outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.CLOSE);
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
        
        //camera.stopStreaming(); // reduces loop times

        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            outtakeSubsystem.outtakeReads();
            intakeSubsystem.intakeReads();

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
        Pose2d poseEstimate = drive.getPoseEstimate();
        switch (currentState) {
            case DELAY:
                outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.CLOSE);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_UP);
                if (GlobalTimer.milliseconds() - autoTimer > 3000){
                    autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                    currentState = AutoState.PRELOAD_DRIVE;
                    if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.LEFT){
                        drive.followTrajectoryAsync(PreloadDrive1);
                        telemetry.addLine("left");
                    } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.CENTER){
                        drive.followTrajectoryAsync(PreloadDrive2);
                        telemetry.addLine("center");
                    } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.RIGHT){
                        drive.followTrajectoryAsync(PreloadDrive3);
                        telemetry.addLine("right");
                    }
                }
                break;
            case PRELOAD_DRIVE:
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_UP);
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                if (!drive.isBusy()){
                    autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
                    currentState = AutoState.OUT_AFTER_PRELOAD_DRIVE;
                }
                break;

            case OUT_AFTER_PRELOAD_DRIVE:
                if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.LEFT){
                    intakeSubsystem.intakeSlideTo(100, intakeSubsystem.intakeSlidePosition,1);
                    outtakeSubsystem.liftTo(140,outtakeSubsystem.liftPosition,1);
                    outtakePreload(poseEstimate);
                } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.CENTER){
                    intakeSubsystem.intakeSlideTo(200, intakeSubsystem.intakeSlidePosition,1);
                    outtakeSubsystem.liftTo(140,outtakeSubsystem.liftPosition,1);
                    outtakePreload(poseEstimate);
                    telemetry.addLine();
                } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.RIGHT){
                    intakeSubsystem.intakeSlideTo(300, intakeSubsystem.intakeSlidePosition,1);
                    outtakeSubsystem.liftTo(140,outtakeSubsystem.liftPosition,1);
                    outtakePreload(poseEstimate);
                }
                if (outtakeSubsystem.liftTargetReached() && intakeSubsystem.intakeSlideTargetReached()){
                    currentState = AutoState.DROP;
                }
                break;

            case OUTTAKE_CONE:

                break;
            case DROP:
                outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.OPEN);
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
        outtakeSubsystem.wristServoState(OuttakeSubsystem.WristServoState.SCORE);
        outtakeSubsystem.pitchToInternalPID(650,1);
    }
}



