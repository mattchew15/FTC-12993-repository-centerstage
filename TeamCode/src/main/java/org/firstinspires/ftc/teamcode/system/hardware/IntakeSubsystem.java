package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem {

RobotHardware robotHardware;

    public enum IntakeSlidesMotorState {
        RETRACT,
        EXTEND,
        LEFT,
        CENTER,
        RIGHT
    }

    public enum IntakeState {
        STOP,
        INTAKE,
        REVERSE,
        DEPOSIT

    }

    public enum BottomRollerState {
        STOP,
        INTAKE,
        REVERSE,
        DEPOSIT
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

    public enum LockState {
        LOCK,
        UNLOCK
    }

    public double degreesToTicks(double degrees) { return degrees / 355; }


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
            case DEPOSIT:
                hardware.extension.setPower(INTAKE_DEPOSIT);
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
            case DEPOSIT:
                hardware.bottomRoller.setPower(BOTTOM_ROLLER_DEPOSIT);
                break;
        }
    }

    public void brushHeightState(BrushHeightState state) {
        switch (state) {
            case INTAKE5:
                hardware.brushHeight.setPosition(degreesToTicks(BRUSH_HEIGHT_5));
                break;
            case INTAKE4:
                hardware.brushHeight.setPosition(degreesToTicks(BRUSH_HEIGHT_4));
                break;
            case INTAKE3:
                hardware.brushHeight.setPosition(degreesToTicks(BRUSH_HEIGHT_3));
                break;
            case INTAKE2:
                hardware.brushHeight.setPosition(degreesToTicks(BRUSH_HEIGHT_2));
                break;
            case INTAKE1:
                hardware.brushHeight.setPosition(degreesToTicks(BRUSH_HEIGHT_1));
                break;
            case DRIVE:
                hardware.brushHeight.setPosition(degreesToTicks(BRUSH_HEIGHT_DRIVE));
                break;
        }
    }

    public void flapState(FlapState state) {
        switch (state) {
            case CLOSE:
                hardware.flap.setPosition(degreesToTicks(FLAP_CLOSE));
                break;
            case OPEN:
                hardware.flap.setPosition(degreesToTicks(FLAP_OPEN));
                break;
        }
    }

    public void lockState(LockState state) {
        switch (state) {
            case LOCK:
                hardware.lock.setPosition(degreesToTicks(LOCK_LOCK));
                break;
            case UNLOCK:
                hardware.lock.setPosition(degreesToTicks(LOCK_UNLOCK));
                break;
        }
    }
}
