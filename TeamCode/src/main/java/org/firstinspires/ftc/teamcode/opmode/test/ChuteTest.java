package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;

@TeleOp(name = "ChuteTest")
public class ChuteTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

        intakeSubsystem.initIntake(hardwareMap);
        intakeSubsystem.intakeHardwareSetup();
        waitForStart();
        double chuteTarget = .91;
        while (!isStopRequested() && opModeIsActive())
        {
            telemetry.addData("ChuteTarget", chuteTarget);
            if (gamepad1.a) chuteTarget = .465;
            if (gamepad1.x) chuteTarget = .91;
            intakeSubsystem.IntakePixelHolderServo.setPosition(chuteTarget);
            telemetry.update();
        }
    }
}