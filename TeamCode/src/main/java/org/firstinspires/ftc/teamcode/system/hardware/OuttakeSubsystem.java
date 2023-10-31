package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.arcrobotics.ftclib.controller.PIDController;

public class OuttakeSubsystem {

    RobotHardware robotHardware;

    public enum ArmServoState {
        READY,
        TRANSFER,
        SCORE,
        SCORE_UP
    }

    public enum PivotState {
        READY,
        DIAGONAL_LEFT,
        DIAGONAL_RIGHT,
        DIAGONAL_LEFT_FLIPPED,
        DIAGONAL_RIGHT_FLIPPED,
        SIDEWAYS_LEFT,
        SIDEWAYS_RIGHT
    }

    public enum WristState {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum ClawState {
        CLOSE,
        NEUTRAL,
        OPEN
    }

    public enum DroneState {
        HOLD,
        RELEASE
    }
    public double degreesToTicks(double degrees) { return degrees / 355; }
    public void ArmServoState(ArmServoState state) {
        switch (state) {
            case READY:
                robotHardware.OuttakeArmServoRight.setPosition(degreesToTicks(ARM_DOWN));
                robotHardware.OuttakeArmServoLeft.setPosition(degreesToTicks(ARM_DOWN));
            break;
            case TRANSFER:
                robotHardware.OuttakeArmServoRight.setPosition(degreesToTicks(ARM_GRAB));
                robotHardware.OuttakeArmServoLeft.setPosition(degreesToTicks(ARM_GRAB));
                break;
            case SCORE:
                robotHardware.OuttakeArmServoRight.setPosition(degreesToTicks(ARM_READY));
                robotHardware.OuttakeArmServoLeft.setPosition(degreesToTicks(ARM_READY));
                break;
            case SCORE_UP:
                robotHardware.OuttakeArmServoRight.setPosition(degreesToTicks(ARM_OUT));
                robotHardware.OuttakeArmServoLeft.setPosition(degreesToTicks(ARM_OUT));
                break;
        }
    }
   public void PivotState(PivotState state) {
        switch (state) {
            case READY:
                robotHardware.PivotServo.setPosition(degreesToTicks(PIVOT_RIGHT));
                break;
            case DIAGONAL_LEFT:
                robotHardware.PivotServo.setPosition(degreesToTicks(PIVOT_LEFT));
                break;
            case DIAGONAL_RIGHT:
                robotHardware.PivotServo.setPosition(degreesToTicks(PIVOT_CENTER));
                break;
            case DIAGONAL_LEFT_FLIPPED:
                robotHardware.PivotServo.setPosition(degreesToTicks(PIVOT_RIGHT));
                break;
            case DIAGONAL_RIGHT_FLIPPED:
                robotHardware.PivotServo.setPosition(degreesToTicks(PIVOT_RIGHT));
                break;
            case SIDEWAYS_LEFT:
                robotHardware.PivotServo.setPosition(degreesToTicks(PIVOT_RIGHT));
                break;
            case SIDEWAYS_RIGHT:
                robotHardware.PivotServo.setPosition(degreesToTicks(PIVOT_RIGHT));
                break;
        }
    }

    public void wristState(WristState state) {
        switch (state) {
            case LEFT:
                robotHardware.WristServo.setPosition(degreesToTicks(WRIST_LEFT));
                break;
            case CENTER:
                robotHardware.WristServo.setPosition(degreesToTicks(WRIST_CENTER));
                break;
            case RIGHT:
                robotHardware.WristServo.setPosition(degreesToTicks(WRIST_RIGHT));
                break;
        }
    }

    public void clawState(ClawState state) {
        switch (state) {
            case CLOSE:
                robotHardware.ClawServo.setPosition(degreesToTicks(CLAW_CLOSE));
                break;
            case NEUTRAL:
                robotHardware.ClawServo.setPosition(degreesToTicks(CLAW_NEUTRAL));
                break;
            case OPEN:
                robotHardware.ClawServo.setPosition(degreesToTicks(CLAW_OPEN));
                break;
        }
    }

    public void droneState(DroneState state) {
        switch (state) {
            case HOLD:
                robotHardware.drone.setPosition(degreesToTicks(DRONE_HOLD));
                break;
            case RELEASE:
                robotHardware.drone.setPosition(degreesToTicks(DRONE_RELEASE));
                break;
        }
    }
}
