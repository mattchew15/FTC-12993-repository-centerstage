package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;

@Config
public class TestIntake extends LinearOpMode
{
    public static double
            INTAKE = 0,
            HEIGHT = 0.4;

    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    @Override
    public void runOpMode() throws InterruptedException
    {

        intakeSubsystem.initIntake(hardwareMap);
        intakeSubsystem.intakeHardwareSetup();
        waitForStart();
        while (!isStopRequested() && opModeIsActive())
        {
            intakeSubsystem.intakeSpin(INTAKE);
            intakeSubsystem.IntakeArmServo.setPosition(HEIGHT);
        }
    }
}
