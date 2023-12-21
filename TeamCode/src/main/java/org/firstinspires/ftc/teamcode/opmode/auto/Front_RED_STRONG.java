package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
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
@Disabled
@Autonomous(name = "Front Red Strong Auto", group = "Autonomous")
public class Front_RED_STRONG extends LinearOpMode {
    // class members
    ElapsedTime GlobalTimer;
    double autoTimer;

    double correctedHeading;


    CameraHardware cameraHardware = new CameraHardware();
    AutoSequences autoSequences = new AutoSequences();
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

    // Define our start pose

    @Override
    public void runOpMode() throws InterruptedException {

        SetAuto.setRedAuto();


        // initialize hardware
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } //

        autoSequences.initAutoHardware(hardwareMap);
        cameraHardware.initWebcam(hardwareMap);


        // functions runs on start

        // Set inital pose
        autoSequences.autoTrajectories.drive.setPoseEstimate(autoSequences.autoTrajectories.startPoseFront);

        // trajectories that aren't changing should all be here

        while (!isStarted()) { // initialization loop
            autoSequences.intializationLoop();
            if (teamPropLocation == 1){
                telemetry.addLine("Front");
            } else if (teamPropLocation == 2){
                telemetry.addLine("Middle");
            } else if (teamPropLocation == 3){
                telemetry.addLine("Back");
            }
            telemetry.update();
        }


        waitForStart();
        if (isStopRequested()) return;


        // runs instantly once
        autoSequences.afterWaitForStart();
        currentState = AutoState.DELAY;
        cameraHardware.closeWebcam(); // reduces loop times

        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            autoSequences.outtakeSubsystem.outtakeReads(false); // might need to change this
            autoSequences.intakeSubsystem.intakeReads(currentState == AutoState.GRAB_OFF_STACK || currentState == AutoState.AFTER_GRAB_OFF_STACK || currentState == AutoState.PLACE_AND_INTAKE);
            poseEstimate = autoSequences.autoTrajectories.drive.getPoseEstimate();

            // Print pose to telemetry
            loopTime.updateLoopTime(telemetry);
            telemetry.addData("numCycles", numCycles);
            telemetry.addData("Auto State", currentState);
            telemetry.addLine("");
            telemetry.addData("liftPosition", autoSequences.outtakeSubsystem.liftPosition);
            telemetry.addData("pitchPosition", autoSequences.outtakeSubsystem.pitchPosition);
            telemetry.addData("intakeSlidePosition", autoSequences.intakeSubsystem.intakeSlidePosition);

            autoSequence();


            xPosition = poseEstimate.getX();              // is in globals rn - might not work idk
            yPosition = poseEstimate.getY();
            headingPosition = poseEstimate.getHeading();

            telemetry.addData("x Position", xPosition);
            telemetry.addData("y Position", yPosition);
            telemetry.addData("heading", headingPosition);
            correctedHeading = angleWrap(Math.toDegrees(headingPosition));


            autoSequences.autoTrajectories.drive.update();
            telemetry.update();
        }

    }

    public void autoSequence(){
        switch (currentState) {
            case DELAY:
                autoSequences.delayState(0);
                if (autoSequences.changeStates){
                    currentState = AutoState.PRELOAD_DRIVE;
                    autoSequences.changeStates = false;
                }

                break;
            case PRELOAD_DRIVE:
                autoSequences.preloadDriveState(1300);
                if (autoSequences.changeStates){
                    currentState = AutoState.PLACE_AND_INTAKE;
                    autoSequences.changeStates = false;
                }
                break;

            case PLACE_AND_INTAKE:
                autoSequences.placeAndIntake();
                if (autoSequences.changeStates){
                    currentState = AutoState.AFTER_PURPLE_DRIVE;
                    autoSequences.changeStates = false;
                }
                break;

            case AFTER_PURPLE_DRIVE:
                autoSequences.afterPurpleDrive();
                if (autoSequences.changeStates){
                    currentState = AutoState.TRANSFER_PIXEL;
                    autoSequences.changeStates = false;
                }
                break;

            case TRANSFER_PIXEL:

                autoSequences.transferPixel();
                if (autoSequences.changeStates){
                    currentState = AutoState.OUTTAKE_PIXEL;
                    autoSequences.changeStates = false;
                }

                break;

            case OUTTAKE_PIXEL:
               autoSequences.outtakePixel(correctedHeading, telemetry);
                if (autoSequences.outtakeChangeStates == 1){
                    currentState = AutoState.DROP;
                } else if (autoSequences.outtakeChangeStates == 2){
                    currentState = AutoState.PARK;
                }
                break;
            case DROP:
                autoSequences.drop();
                if (autoSequences.changeStates){
                    currentState = AutoState.GRAB_OFF_STACK;
                    autoSequences.changeStates = false;
                }
                break;

            // might need to make multiple of these states depending on the trajectory it follows -- again making methods would help alot with this
            case GRAB_OFF_STACK:

                autoSequences.grabOffStack();
                if (autoSequences.changeStates){
                    currentState = AutoState.AFTER_GRAB_OFF_STACK;
                    autoSequences.changeStates = false;
                }
                break;

            // might need to make another state for the colour sensors similar to teleop
            case AFTER_GRAB_OFF_STACK:
               autoSequences.afterGrabOffStack();
                if (autoSequences.changeStates){
                    currentState = AutoState.TRANSFER_PIXEL;
                    autoSequences.changeStates = false;
                }
                break;
            case PARK:
                autoSequences.park();
                if (autoSequences.changeStates){
                    currentState = AutoState.IDLE;
                    autoSequences.changeStates = false;
                }
                break;

            case IDLE:
                autoSequences.idle(telemetry);
                break;
        }
    }
}





