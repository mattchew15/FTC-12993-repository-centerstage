package org.firstinspires.ftc.teamcode.system.base;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.arcrobotics.ftclib.controller.PIDController;

public class IntakeSubsystem {
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
        STOP,
        INTAKE,
        REVERSE
    }

    public enum BrushHeightState {
        INTAKE5,
        INTAKE4,
        INTAKE3,
        INTAKE2,
        INTAKE1,
        DRIVE
    }

    public enum FlapState {
        CLOSE,
        OPEN
    }

    public enum IntakeLockState {
        LOCK,
        UNLOCK
    }

    public double degreesToTicks(double degrees) { return degrees / 355; }

    public void extensionState(ExtensionState state) {
        PIDController controller = new PIDController(EXTENSION_P, EXTENSION_I, EXTENSION_D);
        int pos = hardware.extension.getCurrentPosition();
        dashTelemetry.addData("pos", pos);
        switch (state) {
            case RETRACT:
                double retractPID = controller.calculate(pos, EXTENSION_RETRACT);
                double retractFF = Math. cos(Math. toRadians (EXTENSION_RETRACT / EXTENSION_TICKS_PER_DEGREES)) * EXTENSION_F;
                double retractPower = retractPID + retractFF;
                hardware.extension.setPower(retractPower);
                dashTelemetry.addData("target", EXTENSION_RETRACT);
                break;
            case EXTEND:
                double extendPID = controller.calculate(pos, EXTENSION_EXTEND);
                double extendFF = Math. cos(Math. toRadians (EXTENSION_EXTEND / EXTENSION_TICKS_PER_DEGREES)) * EXTENSION_F;
                double extendPower = extendPID + extendFF;
                hardware.extension.setPower(extendPower);
                dashTelemetry.addData("target", EXTENSION_EXTEND);
                break;
        }
    }

    public void intakeState(IntakeState state) {
        switch (state) {
            case STOP:
                hardware.extension.setPower(INTAKE_STOP);
                break;
            case INTAKE:
                hardware.extension.setPower(INTAKE_INTAKE);
                break;
            case REVERSE:
                hardware.extension.setPower(INTAKE_REVERSE);
                break;
        }
    }

    public void bottomRollerState(BottomRollerState state) {
        switch (state) {
            case STOP:
                hardware.bottomRoller.setPower(BOTTOM_ROLLER_STOP);
                break;
            case INTAKE:
                hardware.bottomRoller.setPower(BOTTOM_ROLLER_INTAKE);
                break;
            case REVERSE:
                hardware.bottomRoller.setPower(BOTTOM_ROLLER_REVERSE);
                break;
        }
    }

    public void brushHeightState(BrushHeightState state) {
        switch (state) {
            case INTAKE5:
                hardware.brushHeight.setPosition(BRUSH_HEIGHT_5);
                break;
            case INTAKE4:
                hardware.brushHeight.setPosition(BRUSH_HEIGHT_4);
                break;
            case INTAKE3:
                hardware.brushHeight.setPosition(BRUSH_HEIGHT_3);
                break;
            case INTAKE2:
                hardware.brushHeight.setPosition(BRUSH_HEIGHT_2);
                break;
            case INTAKE1:
                hardware.brushHeight.setPosition(BRUSH_HEIGHT_1);
                break;
            case DRIVE:
                hardware.brushHeight.setPosition(BRUSH_HEIGHT_DRIVE);
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
