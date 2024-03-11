package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@TeleOp(name="Backdrop Kinematics")
public class BackdropKinTest extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        ElapsedTime timer = new ElapsedTime();
        OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
        DriveBase driveBase = new DriveBase();
        outtakeSubsystem.initOuttake(hardwareMap);
        outtakeSubsystem.hardwareSetup();
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();

        timer.reset();
        timer.startTime();
        waitForStart();
        while (!isStopRequested() && opModeIsActive())
        {
            if (gamepad1.left_trigger > 0.2)
            {
                driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
            else
            {
                outtakeSubsystem.fineAdjustLift(gamepad1.left_stick_y, timer.milliseconds());
            }
        }
    }
}
