package org.firstinspires.ftc.teamcode.system.subsystem;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;

public class OuttakeSubsystem extends SubsystemBase {
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

    public void update(RailState state) {
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

}
