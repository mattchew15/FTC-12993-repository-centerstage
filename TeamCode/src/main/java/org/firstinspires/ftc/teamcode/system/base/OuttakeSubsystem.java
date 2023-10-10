package org.firstinspires.ftc.teamcode.system.base;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.arcrobotics.ftclib.controller.PIDController;

public class OuttakeSubsystem {
    public enum LiftState {
        RETRACT,
        EXTEND
    }

    public enum PitchState {
        ANGLE_30,
        ANGLE_60
    }

    public enum ArmState {
        DOWN,
        GRAB,
        READY,
        OUT
    }

    public enum PivotState {
        LEFT,
        CENTER,
        RIGHT
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

    public void liftState(LiftState state) {
        PIDController controller = new PIDController(LIFT_P, LIFT_I, LIFT_D);
        controller.setPID(LIFT_P, LIFT_I, LIFT_D);
        int pos = hardware.lift.getCurrentPosition();
        dashTelemetry.addData("Lift pos", pos);
        switch (state) {
            case RETRACT:
                double retractPID = controller.calculate(pos, LIFT_RETRACT);
                double retractFF = Math.cos(Math.toRadians(LIFT_RETRACT / LIFT_TICKS_PER_DEGREES)) * LIFT_F;
                double retractPower = retractPID + retractFF;
                hardware.extension.setPower(retractPower);
                dashTelemetry.addData("Lift target", LIFT_RETRACT);
                break;
            case EXTEND:
                double extendPID = controller.calculate(pos, LIFT_EXTEND);
                double extendFF = Math.cos(Math.toRadians(LIFT_EXTEND / LIFT_TICKS_PER_DEGREES)) * LIFT_F;
                double extendPower = extendPID + extendFF;
                hardware.extension.setPower(extendPower);
                dashTelemetry.addData("Lift target", LIFT_EXTEND);
                break;
        }
    }

    public void pitchState(PitchState state) {
        PIDController controller = new PIDController(PITCH_P, PITCH_I, PITCH_D);
        controller.setPID(PITCH_P, PITCH_I, PITCH_D);
        int pos = hardware.pitch.getCurrentPosition();
        dashTelemetry.addData("Pitch pos", pos);
        switch (state) {
            case ANGLE_30:
                double degrees30PID = controller.calculate(pos, PITCH_ANGLE_30);
                double degrees30FF = Math.cos(Math.toRadians(PITCH_ANGLE_30 / PITCH_TICKS_PER_DEGREES)) * PITCH_F;
                double degrees30Power = degrees30PID + degrees30FF;
                hardware.extension.setPower(degrees30Power);
                dashTelemetry.addData("Pitch target", PITCH_ANGLE_30);
                break;
            case ANGLE_60:
                double degrees60PID = controller.calculate(pos, PITCH_ANGLE_60);
                double degrees60FF = Math.cos(Math.toRadians(PITCH_ANGLE_60 / PITCH_TICKS_PER_DEGREES)) * PITCH_F;
                double degrees60Power = degrees60PID + degrees60FF;
                hardware.extension.setPower(degrees60Power);
                dashTelemetry.addData("Pitch target", PITCH_ANGLE_60);
                break;
        }
    }

    public void armState(ArmState state) {
        switch (state) {
            case DOWN:
                hardware.armLeft.setPosition(degreesToTicks(ARM_DOWN));
                hardware.armRight.setPosition(degreesToTicks(ARM_DOWN));
            break;
            case GRAB:
                hardware.armLeft.setPosition(degreesToTicks(ARM_GRAB));
                hardware.armRight.setPosition(degreesToTicks(ARM_GRAB));
                break;
            case READY:
                hardware.armLeft.setPosition(degreesToTicks(ARM_READY));
                hardware.armRight.setPosition(degreesToTicks(ARM_READY));
                break;
            case OUT:
                hardware.armLeft.setPosition(degreesToTicks(ARM_OUT));
                hardware.armRight.setPosition(degreesToTicks(ARM_OUT));
                break;
        }
    }

    public void pivotState(PivotState state) {
        switch (state) {
            case LEFT:
                hardware.pivot.setPosition(degreesToTicks(PIVOT_LEFT));
                break;
            case CENTER:
                hardware.pivot.setPosition(degreesToTicks(PIVOT_CENTER));
                break;
            case RIGHT:
                hardware.pivot.setPosition(degreesToTicks(PIVOT_RIGHT));
                break;
        }
    }

    public void wristState(WristState state) {
        switch (state) {
            case LEFT:
                hardware.wrist.setPosition(degreesToTicks(WRIST_LEFT));
                break;
            case CENTER:
                hardware.wrist.setPosition(degreesToTicks(WRIST_CENTER));
                break;
            case RIGHT:
                hardware.wrist.setPosition(degreesToTicks(WRIST_RIGHT));
                break;
        }
    }

    public void clawState(ClawState state) {
        switch (state) {
            case CLOSE:
                hardware.claw.setPosition(degreesToTicks(CLAW_CLOSE));
                break;
            case NEUTRAL:
                hardware.claw.setPosition(degreesToTicks(CLAW_NEUTRAL));
                break;
            case OPEN:
                hardware.claw.setPosition(degreesToTicks(CLAW_OPEN));
                break;
        }
    }

    public void droneState(DroneState state) {
        switch (state) {
            case HOLD:
                hardware.drone.setPosition(degreesToTicks(DRONE_HOLD));
                break;
            case RELEASE:
                hardware.drone.setPosition(degreesToTicks(DRONE_RELEASE));
                break;
        }
    }
}
