package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
@Autonomous(name="Reset", group = "aTestR")

public class Reset extends OpMode
{
 
    DriveBase drive = new DriveBase();
    @Override
    public void init()
    {
        drive.initDrivebase(hardwareMap);
        drive.drivebaseSetup();
    }

    @Override
    public void loop()
    {
        drive.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}
