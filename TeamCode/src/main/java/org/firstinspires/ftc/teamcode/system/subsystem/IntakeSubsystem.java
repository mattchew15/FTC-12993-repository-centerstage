package org.firstinspires.ftc.teamcode.system.subsystem;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;

public class IntakeSubsystem extends SubsystemBase {
    private RobotHardware robot;

    public enum BottomRollerState {

    }
    public enum BrushHeightState {

    }
    public enum FlapState {
        CLOSE,
        OPEN
    }
    public enum IntakeLockState {
        LOCK,
        UNLOCK
    }

    public void update(BottomRollerState state) {
        switch (state) {
            case //case
                //what to do
        }
    }

    public void update(BrushHeightState state) {
        switch (state) {
            case //case
                    //what to do
        }
    }

    public void update(FlapState state) {
        switch (state) {
            case CLOSE:
                robot.flap.setPosition(FLAP_CLOSE);
                break;
            case OPEN:
                robot.flap.setPosition(FLAP_OPEN);
                break;
        }
    }

    public void update(IntakeLockState state) {
        switch (state) {
            case LOCK:
                robot.intakeLock.setPosition(INTAKE_LOCK_LOCK);
                break;
            case UNLOCK:
                robot.intakeLock.setPosition(INTAKE_LOCK_UNLOCK);
                break;
        }
    }
}
