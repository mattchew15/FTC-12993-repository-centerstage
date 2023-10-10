package org.firstinspires.ftc.teamcode.system.base;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

public class AutoCommand {
    public void updateTimer() {
        autoTimer = GlobalTimer.milliseconds();
        stateTimer = GlobalTimer.milliseconds() - autoTimer;
    }

    public void init() {
        intake.lockState(IntakeSubsystem.LockState.LOCK);
        intake.brushHeightState(IntakeSubsystem.BrushHeightState.DRIVE);
        outtake.armState(OuttakeSubsystem.ArmState.GRAB);
        if (stateTimer > 300) {
            outtake.clawState(OuttakeSubsystem.ClawState.CLOSE);
        }
    }

    public void outtakePurple() {

    }

    public void yellowDrive() {

    }

    public void outtakeYellow() {

    }

    public void stackDrive() {

    }

    public void intakeStack() {

    }

    public void backdropDrive() {

    }

    public void outtakePixels() {

    }

    public void park() {

    }

    public void idle() {

    }
}
