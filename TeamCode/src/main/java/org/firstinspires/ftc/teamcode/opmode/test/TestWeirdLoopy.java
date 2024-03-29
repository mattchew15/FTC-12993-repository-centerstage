package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;

@TeleOp(name="TestLoopy")
public class TestWeirdLoopy extends LinearOpMode
{
    LoopTime loopTime = new LoopTime();


    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();


        while(opModeIsActive())
        {
            /*
            loopTime.delta();
            telemetry.addData("Loop", 1_000_000 / loopTime.getDt());
            telemetry.addData("Hz", loopTime.getHz());

             */
            loopTime.updateLoopTime(telemetry);
            telemetry.update();
        }
    }
}
