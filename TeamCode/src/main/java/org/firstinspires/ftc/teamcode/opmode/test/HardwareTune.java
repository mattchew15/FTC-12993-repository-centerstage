package org.firstinspires.ftc.teamcode.opmode.test;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@TeleOp
public class HardwareTune extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();
        while (opModeIsActive()) {
            /*
            intake.extensionState(IntakeSubsystem.ExtensionState.RETRACT);
            intake.extensionState(IntakeSubsystem.ExtensionState.EXTEND);
            intake.extensionState(IntakeSubsystem.ExtensionState.LEFT);
            intake.extensionState(IntakeSubsystem.ExtensionState.CENTER);
            intake.extensionState(IntakeSubsystem.ExtensionState.RIGHT);

            outtake.liftState(OuttakeSubsystem.LiftState.RETRACT);
            outtake.liftState(OuttakeSubsystem.LiftState.EXTEND);

            outtake.pitchState(OuttakeSubsystem.PitchState.ANGLE_30);
            outtake.pitchState(OuttakeSubsystem.PitchState.ANGLE_60);

            intake.intakeState(IntakeSubsystem.IntakeState.STOP);
            intake.intakeState(IntakeSubsystem.IntakeState.INTAKE);
            intake.intakeState(IntakeSubsystem.IntakeState.REVERSE);
            intake.intakeState(IntakeSubsystem.IntakeState.DEPOSIT);

            intake.bottomRollerState(IntakeSubsystem.BottomRollerState.STOP);
            intake.bottomRollerState(IntakeSubsystem.BottomRollerState.INTAKE);
            intake.bottomRollerState(IntakeSubsystem.BottomRollerState.REVERSE);
            intake.bottomRollerState(IntakeSubsystem.BottomRollerState.DEPOSIT);

            intake.brushHeightState(IntakeSubsystem.BrushHeightState.INTAKE5);
            intake.brushHeightState(IntakeSubsystem.BrushHeightState.INTAKE4);
            intake.brushHeightState(IntakeSubsystem.BrushHeightState.INTAKE3);
            intake.brushHeightState(IntakeSubsystem.BrushHeightState.INTAKE2);
            intake.brushHeightState(IntakeSubsystem.BrushHeightState.INTAKE1);
            intake.brushHeightState(IntakeSubsystem.BrushHeightState.DRIVE);

            intake.flapState(IntakeSubsystem.FlapState.CLOSE);
            intake.flapState(IntakeSubsystem.FlapState.OPEN);

            intake.lockState(IntakeSubsystem.LockState.LOCK);
            intake.lockState(IntakeSubsystem.LockState.UNLOCK);

            outtake.armState(OuttakeSubsystem.ArmState.DOWN);
            outtake.armState(OuttakeSubsystem.ArmState.GRAB);
            outtake.armState(OuttakeSubsystem.ArmState.READY);
            outtake.armState(OuttakeSubsystem.ArmState.OUT);

            outtake.pivotState(OuttakeSubsystem.PivotState.LEFT);
            outtake.pivotState(OuttakeSubsystem.PivotState.CENTER);
            outtake.pivotState(OuttakeSubsystem.PivotState.RIGHT);

            outtake.wristState(OuttakeSubsystem.WristState.LEFT);
            outtake.wristState(OuttakeSubsystem.WristState.CENTER);
            outtake.wristState(OuttakeSubsystem.WristState.RIGHT);

            outtake.clawState(OuttakeSubsystem.ClawState.CLOSE);
            outtake.clawState(OuttakeSubsystem.ClawState.NEUTRAL);
            outtake.clawState(OuttakeSubsystem.ClawState.OPEN);

            outtake.droneState(OuttakeSubsystem.DroneState.HOLD);
            outtake.droneState(OuttakeSubsystem.DroneState.RELEASE);
             */
        }
    }
}
