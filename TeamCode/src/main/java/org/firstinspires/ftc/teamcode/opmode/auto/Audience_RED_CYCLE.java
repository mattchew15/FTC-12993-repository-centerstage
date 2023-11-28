/*
package org.firstinspires.ftc.teamcode.opmode.auto;


import static org.firstinspires.ftc.teamcode.system.vision.YCrCbRedTeamPropDetectorPipeline.RED_POSITION;

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

@Autonomous(name = "Audience red cycle")
public class Audience_RED_CYCLE extends LinearOpMode {
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

    Trajectory PreloadDrive1, PreloadDrive2, PreloadDrive3, Reverse;


    // Define our start pose

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap); // road drive class
        SetAuto.setRedAuto();

        startPose = new Pose2d(-38, -59, Math.toRadians(180));


        PreloadDrive1 = drive.trajectoryBuilder(startPose) // Left
                .lineToLinearHeading(new Pose2d(-38, -33, Math.toRadians(170)))
                .build();
        PreloadDrive2 = drive.trajectoryBuilder(startPose) // Middle
                .lineToLinearHeading(new Pose2d(-35, -30, Math.toRadians(105)))
                .build();
        PreloadDrive3 = drive.trajectoryBuilder(startPose) // Right
                .lineToLinearHeading(new Pose2d(-32, -33, Math.toRadians(10)))
                .build();
        Reverse = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(1.5)
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
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
            telemetry.update();
        }


        waitForStart();
        if (isStopRequested()) return;


        // runs instantly once
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        autoTimer = GlobalTimer.milliseconds();

        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING); // just so we don't have an extra write during the loop

        currentState = AutoState.DELAY; // this go here?
        autoTimer = 0;
        numCycles = 0;

        //camera.stopStreaming(); // reduces loop times

        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            intakeSubsystem.intakeReads(false);

            // Print pose to telemetry
            loopTime.updateLoopTime(telemetry);
            telemetry.addData("Auto State", currentState);
            telemetry.addLine("");


            autoSequence();


            //xPosition = poseEstimate.getX();              // could put the x,y and heading into globals
            //yPosition = poseEstimate.getY();
            //headingPosition = poseEstimate.getHeading();

            drive.update();
            telemetry.update();
        }

    }

    public void autoSequence() {
        Pose2d poseEstimate = drive.getPoseEstimate();
        switch (currentState) {
            case DELAY:
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);

                if (GlobalTimer.milliseconds() - autoTimer > 3000) {
                    GlobalTimer.reset(); // reset timer not rly needed here
                    currentState = AutoState.PRELOAD_DRIVE;

                    if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.LEFT) {
                        drive.followTrajectoryAsync(PreloadDrive1);
                        telemetry.addLine("left");
                    } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.CENTER) {
                        drive.followTrajectoryAsync(PreloadDrive2);
                        telemetry.addLine("center");
                    } else if (RED_POSITION == YCrCbRedTeamPropDetectorPipeline.TeamPropPosition.RIGHT) {
                        drive.followTrajectoryAsync(PreloadDrive3);
                        telemetry.addLine("right");
                    }
                }
                break;
            case PRELOAD_DRIVE:
                if (GlobalTimer.milliseconds() > 4000) {
                    currentState = AutoState.IDLE;
                    GlobalTimer.reset();
                }
                if (GlobalTimer.milliseconds() > 2750)
                {
                    intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP);
                }
                break;

            case IDLE:
                currentState = AutoState.DROP;
                drive.followTrajectoryAsync(Reverse);

                //reset outtake servos
                break;
            case DROP:
                telemetry.addLine("WWWWWWWWWWW");
                break;

        }
    }
}

 */