package org.firstinspires.ftc.teamcode.system.base;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;

public class OuttakeSubsystem {
    private RobotHardware hardware;

    public enum RailState {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum ArmState {
        DOWN,
        LIFT,
        UP,
        LEVEL
    }

    public enum PivotState {
        LEFT,
        VERTICAL,
        RIGHT
    }

    public enum WristState {
        LEFT,
        STRAIGHT,
        RIGHT
    }

    public enum ClawState {
        CLOSE,
        NEUTRAL,
        OPEN
    }

    public enum OuttakeLockState {
        LOCK,
        UNLOCK
    }

    public enum DroneState {
        HOLD,
        RELEASE
    }

    public void RailState(RailState state) {
        switch (state) {
            case LEFT:
                hardware.rail.setPosition(RAIL_LEFT);
                break;
            case CENTER:
                hardware.rail.setPosition(RAIL_CENTER);
                break;
            case RIGHT:
                hardware.rail.setPosition(RAIL_RIGHT);
                break;
        }
    }

    public void ArmState(ArmState state) {
        switch (state) {
            case DOWN:
                hardware.armLeft.setPosition(ARM_LEFT_DOWN);
                hardware.armRight.setPosition(ARM_RIGHT_DOWN);
            break;
            case LIFT:
                hardware.armLeft.setPosition(ARM_LEFT_LIFT);
                hardware.armRight.setPosition(ARM_RIGHT_LIFT);
                break;
            case UP:
                hardware.armLeft.setPosition(ARM_LEFT_UP);
                hardware.armRight.setPosition(ARM_RIGHT_UP);
                break;
            case LEVEL:
                hardware.armLeft.setPosition(ARM_LEFT_LEVEL);
                hardware.armRight.setPosition(ARM_RIGHT_LEVEL);
                break;
        }
    }

    public void PivotState(PivotState state) {
        switch (state) {
            case LEFT:
                hardware.rail.setPosition(PIVOT_LEFT);
                break;
            case VERTICAL:
                hardware.rail.setPosition(PIVOT_VERTICAL);
                break;
            case RIGHT:
                hardware.rail.setPosition(PIVOT_RIGHT);
                break;
        }
    }

    public void WristState(WristState state) {
        switch (state) {
            case LEFT:
                hardware.rail.setPosition(WRIST_LEFT);
                break;
            case STRAIGHT:
                hardware.rail.setPosition(WRIST_STRAIGHT);
                break;
            case RIGHT:
                hardware.rail.setPosition(WRIST_RIGHT);
                break;
        }
    }

    public void ClawState(ClawState state) {
        switch (state) {
            case CLOSE:
                hardware.rail.setPosition(CLAW_CLOSE);
                break;
            case NEUTRAL:
                hardware.rail.setPosition(CLAW_NEUTRAL);
                break;
            case OPEN:
                hardware.rail.setPosition(CLAW_OPEN);
                break;
        }
    }

    public void OuttakeLockState(OuttakeLockState state) {
        switch (state) {
            case LOCK:
                hardware.rail.setPosition(OUTTAKE_LOCK_LOCK);
                break;
            case UNLOCK:
                hardware.rail.setPosition(OUTTAKE_LOCK_UNLOCK);
                break;
        }
    }

    public void DroneState(DroneState state) {
        switch (state) {
            case HOLD:
                hardware.drone.setPosition(DRONE_HOLD);
                break;
            case RELEASE:
                hardware.drone.setPosition(DRONE_RELEASE);
                break;
        }
    }
}
