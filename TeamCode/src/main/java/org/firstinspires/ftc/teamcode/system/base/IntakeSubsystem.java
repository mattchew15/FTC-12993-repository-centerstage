package org.firstinspires.ftc.teamcode.system.base;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class IntakeSubsystem {
    private final  Telemetry dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

    public void extensionState(ExtensionState state) {
        PIDController controller = new PIDController(EXTENSION_P, EXTENSION_I, EXTENSION_D);
        int pos = hardware.extension.getCurrentPosition();
        switch (state) {
            case RETRACT:
                double retractPID = controller.calculate(pos, EXTENSION_RETRACT);
                double retractFF = Math. cos(Math. toRadians (EXTENSION_RETRACT / EXTENSION_TICKS_IN_DEGREES)) * EXTENSION_F;
                double retractPower = retractPID + retractFF;
                hardware.extension.setPower(retractPower);
                break;
            case EXTEND:
                double extendPID = controller.calculate(pos, EXTENSION_EXTEND);
                double extendFF = Math. cos(Math. toRadians (EXTENSION_EXTEND / EXTENSION_TICKS_IN_DEGREES)) * EXTENSION_F;
                double extendPower = extendPID + extendFF;
                hardware.extension.setPower(extendPower);
                break;
        }
    }

    public void intakeState(IntakeState state) {
        switch (state) {
            case STOP:
                hardware.extension.setPower(0);
                break;
            case INTAKE:
                hardware.extension.setPower(1);
                break;
            case REVERSE:
                hardware.extension.setPower(-1);
                break;
        }
    }

    public void bottomRollerState(BottomRollerState state) {
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

    public void brushHeightState(BrushHeightState state) {
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

    public void flapState(FlapState state) {
        switch (state) {
            case CLOSE:
                hardware.flap.setPosition(FLAP_CLOSE);
                break;
            case OPEN:
                hardware.flap.setPosition(FLAP_OPEN);
                break;
        }
    }

    public void intakeLockState(IntakeLockState state) {
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
