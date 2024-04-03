package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.CameraHardware;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.*;
@Disabled
@Autonomous(name = "Test them middle trajectories bro", group = "Autonomous")
public class TEST_MIDDLE_TRAJECTORIES extends LinearOpMode {

    int numCycleForDifferentLane = 0;

    //Accessories
    AutoSequences auto = new AutoSequences(telemetry);
    LoopTime loopTime = new LoopTime();
    // this works because the parameters being passed in don't change throughout the opmode
    // same cycle numbers
    AutoRail railLogic = new AutoRail(numCycleForDifferentLane,0, auto, false,1);
    AutoPivot pivotLogic = new AutoPivot(numCycleForDifferentLane,0, auto, telemetry,1);


    enum AutoState{
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

    double newX, newY;
    double offsetX, offsetY;

    @Override
    public void runOpMode() throws InterruptedException {

        SetAuto.setRedAuto();

        //cameraHardware.initBackWebcamVP(hardwareMap, telemetry);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            module.clearBulkCache();
        } //

        auto.initAutoHardware(hardwareMap,this);
        // auto.cameraHardware.initBackWebcamVP(hardwareMap);

        // trajectories that aren't changing should all be here
        while (!isStarted()) { // initialization loop
            auto.intializationLoop();
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
        auto.afterWaitForStart();
        //  auto.cameraHardware.pauseBackWebcam();
        currentState = AutoState.DELAY;

        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
                module.clearBulkCache();
            }

            auto.mainAutoLoop(
                    currentState == AutoState.OUTTAKE_PIXEL && xPosition > 24,
                    currentState == AutoState.GRAB_OFF_STACK || currentState == AutoState.AFTER_GRAB_OFF_STACK || currentState == AutoState.PLACE_AND_INTAKE,
                    currentState != AutoState.PRELOAD_DRIVE && currentState != AutoState.OUTTAKE_PIXEL);


            autoSequence();
            loopTime.delta();
            telemetry.addData("xPosition", xPosition);
            telemetry.addData("yPosition", yPosition);
            telemetry.addData("headingPosition", headingPosition);
            telemetry.addData("drive is busy!!!!!", auto.autoTrajectories.drive.isBusy());
            telemetry.addData("LoopTime", loopTime.getDt() / 1_000_000);
            telemetry.addData("Hz", loopTime.getHz());
            telemetry.addData("Auto State", currentState);
            telemetry.addData("intakeSlidePosition", auto.intakeSubsystem.intakeSlidePosition);

            telemetry.update();

        }
    }

    public void autoSequence(){
        auto.outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
        // auto.goToPark(currentState == AutoState.IDLE,1);
        auto.intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING);
        switch (currentState) {
            case DELAY:
                if (teamPropLocation == 1){
                    auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive1Front);
                    telemetry.addLine("left");
                } else if (true){
                    auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive2FrontStage);
                    telemetry.addLine("center");
                } else if (teamPropLocation == 3){
                    auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive3Front);
                    telemetry.addLine("right");
                }
                currentState = AutoState.PRELOAD_DRIVE;
                break;
            case PRELOAD_DRIVE:
                currentState = AutoState.PLACE_AND_INTAKE;
                break;
            case PLACE_AND_INTAKE:
                if (!auto.autoTrajectories.drive.isBusy()){
                    Trajectory startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple2;
                    auto.autoTrajectories.drive.followTrajectoryAsync(startDrive);
                    currentState = AutoState.TRANSFER_PIXEL;
                }
                break;

            case TRANSFER_PIXEL:
                currentState = AutoState.OUTTAKE_PIXEL;
                break;

            case OUTTAKE_PIXEL:
                // apriltag relocalization, compare offsets for this and the telemetry of the odo
                if (auto.cameraHardware.getNewPose2(poseEstimate, 1, telemetry))
                {
                    Pose2d pose = auto.cameraHardware.getNewPose();
                    newX = pose.getX();
                    newY = pose.getY();
                }
                offsetX = xPosition - newX;
                offsetY = yPosition - newY;
                //telemetry.addData("X offset", newX);
                //telemetry.addData("Y offset", newY);
                telemetry.addData("OffsetX", offsetX);
                telemetry.addData("OffsetY", offsetY);
                //outtaking lengths for each cycle
               /* if (!auto.autoTrajectories.drive.isBusy()){
                    Trajectory intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectory(poseEstimate,25,5,145,1,0);
                    auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectory);

                    currentState = AutoState.DROP;
                }*/
                break;
            case DROP:
                currentState = AutoState.GRAB_OFF_STACK;
                break;
            case GRAB_OFF_STACK:
                if (!auto.autoTrajectories.drive.isBusy()){
                    currentState = AutoState.AFTER_GRAB_OFF_STACK;
                    Trajectory outtakeTrajectory;
                    //outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2TrussTrajectory(poseEstimate,14,-176,4);
                    outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromAngleTurnEndTrajectory(poseEstimate,12,4,4, 1);
                    auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);
                }
                break;
            case AFTER_GRAB_OFF_STACK:
                currentState = AutoState.TRANSFER_PIXEL;
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


