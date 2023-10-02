package org.firstinspires.ftc.teamcode.system.base;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;

public class OuttakeSubsystem {
    private RobotHardware robot;

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
                robot.rail.setPosition(RAIL_LEFT);
                break;
            case CENTER:
                robot.rail.setPosition(RAIL_CENTER);
                break;
            case RIGHT:
                robot.rail.setPosition(RAIL_RIGHT);
                break;
        }
    }

    public void ArmState(ArmState state) {
        switch (state) {
            case DOWN:
                robot.armLeft.setPosition(ARM_LEFT_DOWN);
                robot.armRight.setPosition(ARM_RIGHT_DOWN);
            break;
            case LIFT:
                robot.armLeft.setPosition(ARM_LEFT_LIFT);
                robot.armRight.setPosition(ARM_RIGHT_LIFT);
                break;
            case UP:
                robot.armLeft.setPosition(ARM_LEFT_UP);
                robot.armRight.setPosition(ARM_RIGHT_UP);
                break;
            case LEVEL:
                robot.armLeft.setPosition(ARM_LEFT_LEVEL);
                robot.armRight.setPosition(ARM_RIGHT_LEVEL);
                break;
        }
    }

    public void PivotState(PivotState state) {
        switch (state) {
            case LEFT:
                robot.rail.setPosition(PIVOT_LEFT);
                break;
            case VERTICAL:
                robot.rail.setPosition(PIVOT_VERTICAL);
                break;
            case RIGHT:
                robot.rail.setPosition(PIVOT_RIGHT);
                break;
        }
    }

    public void WristState(WristState state) {
        switch (state) {
            case LEFT:
                robot.rail.setPosition(WRIST_LEFT);
                break;
            case STRAIGHT:
                robot.rail.setPosition(WRIST_STRAIGHT);
                break;
            case RIGHT:
                robot.rail.setPosition(WRIST_RIGHT);
                break;
        }
    }

    public void ClawState(ClawState state) {
        switch (state) {
            case CLOSE:
                robot.rail.setPosition(CLAW_CLOSE);
                break;
            case NEUTRAL:
                robot.rail.setPosition(CLAW_NEUTRAL);
                break;
            case OPEN:
                robot.rail.setPosition(CLAW_OPEN);
                break;
        }
    }

    public void OuttakeLockState(OuttakeLockState state) {
        switch (state) {
            case LOCK:
                robot.rail.setPosition(OUTTAKE_LOCK_LOCK);
                break;
            case UNLOCK:
                robot.rail.setPosition(OUTTAKE_LOCK_UNLOCK);
                break;
        }
    }

    public void DroneState(DroneState state) {
        switch (state) {
            case HOLD:
                robot.drone.setPosition(DRONE_HOLD);
                break;
            case RELEASE:
                robot.drone.setPosition(DRONE_RELEASE);
                break;
        }
    }
}
