/*
package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

@Autonomous
public class BlueBackSimpleAuto extends LinearOpMode {
    private enum AutoState {
        DELAY,
        BACKDROP_DRIVE,
        OUTTAKE_PRELOAD,
        PARK,
        IDLE
    }

    @Override
    public void runOpMode() {
        SetAuto.setBlueAuto();
        SetAuto.setBackAuto();
        hardware.initWebcam(hardwareMap);

        // Gets position to park
        BLUE_POSITION = hardware.getBluePosition();

        AutoState currentState = AutoState.DELAY;

        // Reset timer
        GlobalTimer.reset();
        command.updateTimer();

        while (!isStarted()) {
            print.telemetryTeamProp();
            telemetry.update();
            command.init();
        }

        hardware.initAprilTag(hardwareMap);

        waitForStart();
        // Reset timer
        GlobalTimer.reset();
        command.updateTimer();

        while (opModeIsActive() && !isStopRequested()) {
            print.telemetryTeamProp();
            print.telemetryAprilTag();
            telemetry.addData("Current State", currentState);
            telemetry.update();

            // Main state machine
            switch (currentState) {
                case DELAY:
                    outtake.armState(OuttakeSubsystem.ArmState.READY);
                    if (stateTimer > 5000) {
                        currentState = AutoState.BACKDROP_DRIVE;
                    }
                    command.updateTimer();
                    break;

                case BACKDROP_DRIVE:
                    if (BLUE_POSITION == BLUE_LEFT) {
                        // Drive to left pos
                        outtake.pivotState(OuttakeSubsystem.PivotState.LEFT);
                        outtake.wristState(OuttakeSubsystem.WristState.LEFT);
                        intake.extensionState(IntakeSubsystem.ExtensionState.LEFT);
                    } else if (BLUE_POSITION == BLUE_CENTER) {
                        // Drive to center pos
                        outtake.pivotState(OuttakeSubsystem.PivotState.CENTER);
                        outtake.wristState(OuttakeSubsystem.WristState.CENTER);
                        intake.extensionState(IntakeSubsystem.ExtensionState.CENTER);
                    } else if (BLUE_POSITION == BLUE_RIGHT) {
                        // Drive to right pos
                        outtake.pivotState(OuttakeSubsystem.PivotState.RIGHT);
                        outtake.wristState(OuttakeSubsystem.WristState.RIGHT);
                        intake.extensionState(IntakeSubsystem.ExtensionState.RIGHT);
                    }
                    if (true) { // drive to position

                    }
                    outtake.armState(OuttakeSubsystem.ArmState.OUT);
                    outtake.pitchState(OuttakeSubsystem.PitchState.ANGLE_30);
                    outtake.liftState(OuttakeSubsystem.LiftState.EXTEND);
                    currentState = AutoState.OUTTAKE_PRELOAD;
                    break;

                case OUTTAKE_PRELOAD:
                    intake.intakeState(IntakeSubsystem.IntakeState.DEPOSIT);
                    outtake.clawState(OuttakeSubsystem.ClawState.OPEN);
                    currentState = AutoState.PARK;
                    break;

                case PARK:
                    // Drive to park
                    currentState = AutoState.IDLE;
                    break;

                case IDLE:
                    outtake.clawState(OuttakeSubsystem.ClawState.OPEN);
                    outtake.armState(OuttakeSubsystem.ArmState.DOWN);
                    outtake.pitchState(OuttakeSubsystem.PitchState.ANGLE_60);
                    outtake.liftState(OuttakeSubsystem.LiftState.RETRACT);
                    intake.extensionState(IntakeSubsystem.ExtensionState.RETRACT);
                    break;
            }

            if ((GlobalTimer.milliseconds() > 27500) && currentState != AutoState.IDLE){
                currentState = AutoState.PARK;
            }
        }
    }
}

 */
