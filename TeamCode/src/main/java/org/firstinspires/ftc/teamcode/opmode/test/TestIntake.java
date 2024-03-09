package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;

@Config
@TeleOp(name="IntakeTest")
public class TestIntake extends LinearOpMode {
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public static double
            INTAKE_MOTOR = 0,
            INTAKE_ARM_POS = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeSubsystem.initIntake(hardwareMap);
        intakeSubsystem.intakeHardwareSetup();

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            intakeSubsystem.IntakeMotor.setPower(INTAKE_MOTOR);
            intakeSubsystem.IntakeArmServo.setPosition(INTAKE_ARM_POS);
        }
    }
}
