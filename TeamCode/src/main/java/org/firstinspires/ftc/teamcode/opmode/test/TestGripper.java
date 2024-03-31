package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@TeleOp
public class TestGripper extends LinearOpMode
{
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    @Override
    public void runOpMode() throws InterruptedException
    {
        outtakeSubsystem.initOuttake(hardwareMap);
        outtakeSubsystem.hardwareSetup();
        OuttakeSubsystem.GripperServoState state = OuttakeSubsystem.GripperServoState.OPEN;
        waitForStart();
        while (opModeIsActive())
        {
            if (gamepad1.a)
            {
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
            }
            if (gamepad1.b)
            {
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
            }
        }
    }
}
