package org.firstinspires.ftc.teamcode.system.base;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;

@Config
public class IntakeSubsystem {
    private RobotHardware robot;

    public enum BottomRollerState {
        INTAKE5,
        INTAKE4,
        INTAKE3,
        INTAKE2,
        INTAKE1
    }

    public enum BrushHeightState {
        INTAKE5,
        INTAKE4,
        INTAKE3,
        INTAKE2,
        INTAKE1
    }

    public enum FlapState {
        CLOSE,
        OPEN
    }

    public enum IntakeLockState {
        LOCK,
        UNLOCK
    }

    public enum ExtensionState {
        IN,
        OUT
    }

    public enum IntakeState {
        STOP,
        INTAKE,
        REVERSE
    }

    public void bottomRoller(BottomRollerState state) {
        switch (state) {
            case INTAKE5:
                robot.bottomRoller.setPosition(BOTTOM_ROLLER_5);
                break;
            case INTAKE4:
                robot.bottomRoller.setPosition(BOTTOM_ROLLER_4);
                break;
            case INTAKE3:
                robot.bottomRoller.setPosition(BOTTOM_ROLLER_3);
                break;
            case INTAKE2:
                robot.bottomRoller.setPosition(BOTTOM_ROLLER_2);
                break;
            case INTAKE1:
                robot.bottomRoller.setPosition(BOTTOM_ROLLER_1);
                break;
        }
    }

    public void brushHeight(BrushHeightState state) {
        switch (state) {
            case INTAKE5:
                robot.bottomRoller.setPosition(BRUSH_HEIGHT_5);
                break;
            case INTAKE4:
                robot.bottomRoller.setPosition(BRUSH_HEIGHT_4);
                break;
            case INTAKE3:
                robot.bottomRoller.setPosition(BRUSH_HEIGHT_3);
                break;
            case INTAKE2:
                robot.bottomRoller.setPosition(BRUSH_HEIGHT_2);
                break;
            case INTAKE1:
                robot.bottomRoller.setPosition(BRUSH_HEIGHT_1);
                break;
        }
    }

    public void flap(FlapState state) {
        switch (state) {
            case CLOSE:
                robot.flap.setPosition(FLAP_CLOSE);
                break;
            case OPEN:
                robot.flap.setPosition(FLAP_OPEN);
                break;
        }
    }

    public void intakeLock(IntakeLockState state) {
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
