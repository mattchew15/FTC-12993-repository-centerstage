package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@Config
@TeleOp(group = "Tune")
public class ServoTuner extends LinearOpMode {

    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();

    public static double
            INTAKE_ARM_POS = 0.48,
            INTAKE_FLAP_POS = 0.57,
            INTAKE_CLIP_POS = 0.75,
            INTAKE_PIXEL_HOLDER_POS = 0.5,
            OUTTAKE_ARM_POS = 0.92,
            MINI_TURRET_POS = 0.489,
            PIVOT_POS = 0.53,
            WRIST_POS = 0.26,
            CLAW_POS = 0.6;

    @Override
    public void runOpMode() {

        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            intakeSubsystem.IntakeArmServo.setPosition(INTAKE_ARM_POS);
            intakeSubsystem.IntakeFlapServo.setPosition(INTAKE_FLAP_POS);
            intakeSubsystem.IntakeClipServo.setPosition(INTAKE_CLIP_POS);
            intakeSubsystem.IntakePixelHolderServo.setPosition(INTAKE_PIXEL_HOLDER_POS);
            outtakeSubsystem.OuttakeArmServoLeft.setPosition(OUTTAKE_ARM_POS);
            outtakeSubsystem.OuttakeArmServoRight.setPosition(OUTTAKE_ARM_POS);
            outtakeSubsystem.MiniTurretServo.setPosition(MINI_TURRET_POS);
            outtakeSubsystem.PivotServo.setPosition(PIVOT_POS);
            outtakeSubsystem.WristServo.setPosition(WRIST_POS);
            outtakeSubsystem.ClawServo.setPosition(CLAW_POS);
        }
    }
}
