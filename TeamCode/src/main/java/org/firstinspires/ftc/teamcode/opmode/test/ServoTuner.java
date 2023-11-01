package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;

@Config
@TeleOp
public class ServoTuner extends LinearOpMode {
    RobotHardware robotHardware = new RobotHardware();

    public static double
            INTAKE_ARM_POS = 0.48,
            INTAKE_FLAP_POS = 0.57,
            INTAKE_CLIP_POS = 0.5,
            OUTTAKE_ARM_POS = 0.22,
            MINI_TURRET_POS = 0.489,
            PIVOT_POS = 0.53,
            WRIST_POS = 0.63,
            CLAW_POS = 0.4;

    @Override
    public void runOpMode() {

        robotHardware.initializeHardware(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            robotHardware.IntakeArmServo.setPosition(INTAKE_ARM_POS);
            robotHardware.IntakeFlapServo.setPosition(INTAKE_FLAP_POS);
            robotHardware.IntakeClipServo.setPosition(INTAKE_CLIP_POS);
            robotHardware.OuttakeArmServoLeft.setPosition(OUTTAKE_ARM_POS);
            robotHardware.OuttakeArmServoRight.setPosition(OUTTAKE_ARM_POS);
            robotHardware.MiniTurretServo.setPosition(MINI_TURRET_POS);
            robotHardware.PivotServo.setPosition(PIVOT_POS);
            robotHardware.WristServo.setPosition(WRIST_POS);
            robotHardware.ClawServo.setPosition(CLAW_POS);
        }
    }
}
