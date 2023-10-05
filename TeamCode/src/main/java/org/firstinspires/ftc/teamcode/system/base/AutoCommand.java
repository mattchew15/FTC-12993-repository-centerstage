package org.firstinspires.ftc.teamcode.system.base;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

public class AutoCommand {
    enum AutoState {
        DELAY,
        PRELOAD_DRIVE,
        OUTTAKE_CONE_AFTER_PRELOAD,
        OUTTAKE_CONE,
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

    /*
    BACK SIDE:
    drive to spike mark
    score purple pixel
    drive to backdrop
    score yellow pixel
    loop{
    drive to stack
    intake 2 pixels from stack (change height)
    drive to backdrop
    deposit 2 pixels
    }
    park
    */

    public AutoState currentState = AutoState.DELAY;

    public void updateTimer() {
        autoTimer = GlobalTimer.milliseconds();
        stateTimer = GlobalTimer.milliseconds() - autoTimer;
    }

    public void init() {
        intake.intakeLockState(IntakeSubsystem.IntakeLockState.UNLOCK);
        intake.brushHeightState(IntakeSubsystem.BrushHeightState.DRIVE);
        outtake.clawState(OuttakeSubsystem.ClawState.CLOSE);
        if (stateTimer == 300) {
            outtake.armState(OuttakeSubsystem.ArmState.READY);
        }
    }

    public void preloadDrive() {

    }

    public void outtakePurple() {

    }

    public void intake() {

    }

    public void transfer() {

    }

    public void outtakeYellow() {

    }

    public void outtakeWhite() {

    }
}
