package org.firstinspires.ftc.teamcode.system.base;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeSubsystem {
    private final Telemetry dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    public enum LiftState {
        RETRACT,
        EXTEND
    }

    public enum PitchState {
        DEGREES_30,
        DEGREES_60
    }

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

    public void liftState(LiftState state) {
        PIDController controller = new PIDController(LIFT_P, LIFT_I, LIFT_D);
        int pos = hardware.lift.getCurrentPosition();
        switch (state) {
            case RETRACT:
                double retractPID = controller.calculate(pos, LIFT_RETRACT);
                double retractFF = Math.cos(Math.toRadians(LIFT_RETRACT / LIFT_TICKS_IN_DEGREES)) * LIFT_F;
                double retractPower = retractPID + retractFF;
                hardware.extension.setPower(retractPower);
                break;
            case EXTEND:
                double extendPID = controller.calculate(pos, LIFT_EXTEND);
                double extendFF = Math.cos(Math.toRadians(LIFT_EXTEND / LIFT_TICKS_IN_DEGREES)) * LIFT_F;
                double extendPower = extendPID + extendFF;
                hardware.extension.setPower(extendPower);
                break;
        }
    }

    public void pitchState(PitchState state) {
        PIDController controller = new PIDController(PITCH_P, PITCH_I, PITCH_D);
        int pos = hardware.pitch.getCurrentPosition();
        switch (state) {
            case DEGREES_30:
                double degrees30PID = controller.calculate(pos, PITCH_DEGREES_30);
                double degrees30FF = Math.cos(Math.toRadians(PITCH_DEGREES_30 / PITCH_TICKS_IN_DEGREES)) * PITCH_F;
                double degrees30Power = degrees30PID + degrees30FF;
                hardware.extension.setPower(degrees30Power);
                break;
            case DEGREES_60:
                double degrees60PID = controller.calculate(pos, PITCH_DEGREES_60);
                double degrees60FF = Math.cos(Math.toRadians(PITCH_DEGREES_60 / PITCH_TICKS_IN_DEGREES)) * PITCH_F;
                double degrees60Power = degrees60PID + degrees60FF;
                hardware.extension.setPower(degrees60Power);
                break;
        }
    }

    public void railState(RailState state) {
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

    public void armState(ArmState state) {
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

    public void pivotState(PivotState state) {
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

    public void wristState(WristState state) {
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

    public void clawState(ClawState state) {
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

    public void outtakeLockState(OuttakeLockState state) {
        switch (state) {
            case LOCK:
                hardware.rail.setPosition(OUTTAKE_LOCK_LOCK);
                break;
            case UNLOCK:
                hardware.rail.setPosition(OUTTAKE_LOCK_UNLOCK);
                break;
        }
    }

    public void droneState(DroneState state) {
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
