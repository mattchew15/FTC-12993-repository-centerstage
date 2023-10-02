package org.firstinspires.ftc.teamcode.system.base;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;

import java.security.cert.Extension;

@Config
public class IntakeSubsystem {
    private RobotHardware hardware;
    private final PIDController extensionController = new PIDController(EXTENSION_P, EXTENSION_I, EXTENSION_D);

    public enum ExtensionState {
        RETRACT,
        EXTEND
    }

    public enum IntakeState {
        STOP,
        INTAKE,
        REVERSE
    }

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

    public void extension(ExtensionState state) {
        switch (state) {
            case RETRACT:

                break;
            case EXTEND:

                break;
        }
    }

    public void intake(IntakeState state) {
        switch (state) {
            case STOP:

                break;
            case INTAKE:

                break;
            case REVERSE:

                break;
        }
    }

    public void bottomRoller(BottomRollerState state) {
        switch (state) {
            case INTAKE5:
                hardware.bottomRoller.setPosition(BOTTOM_ROLLER_5);
                break;
            case INTAKE4:
                hardware.bottomRoller.setPosition(BOTTOM_ROLLER_4);
                break;
            case INTAKE3:
                hardware.bottomRoller.setPosition(BOTTOM_ROLLER_3);
                break;
            case INTAKE2:
                hardware.bottomRoller.setPosition(BOTTOM_ROLLER_2);
                break;
            case INTAKE1:
                hardware.bottomRoller.setPosition(BOTTOM_ROLLER_1);
                break;
        }
    }

    public void brushHeight(BrushHeightState state) {
        switch (state) {
            case INTAKE5:
                hardware.bottomRoller.setPosition(BRUSH_HEIGHT_5);
                break;
            case INTAKE4:
                hardware.bottomRoller.setPosition(BRUSH_HEIGHT_4);
                break;
            case INTAKE3:
                hardware.bottomRoller.setPosition(BRUSH_HEIGHT_3);
                break;
            case INTAKE2:
                hardware.bottomRoller.setPosition(BRUSH_HEIGHT_2);
                break;
            case INTAKE1:
                hardware.bottomRoller.setPosition(BRUSH_HEIGHT_1);
                break;
        }
    }

    public void flap(FlapState state) {
        switch (state) {
            case CLOSE:
                hardware.flap.setPosition(FLAP_CLOSE);
                break;
            case OPEN:
                hardware.flap.setPosition(FLAP_OPEN);
                break;
        }
    }

    public void intakeLock(IntakeLockState state) {
        switch (state) {
            case LOCK:
                hardware.intakeLock.setPosition(INTAKE_LOCK_LOCK);
                break;
            case UNLOCK:
                hardware.intakeLock.setPosition(INTAKE_LOCK_UNLOCK);
                break;
        }
    }
}
