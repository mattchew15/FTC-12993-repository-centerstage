package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.accessory.Log;
import org.firstinspires.ftc.teamcode.system.accessory.PID;
import org.firstinspires.ftc.teamcode.system.accessory.FeedFoward;

@TeleOp(name = "LOG  ")
public class LogTest extends LinearOpMode
{
    Log log;
    double i;
    public static double LiftKp = 0.015, LiftKi = 0.0001, LiftKd = 0.00006, LiftIntegralSumLimit = 10, LiftKf = 0;
    PID pid = new PID(LiftKp,LiftKi,LiftKd,LiftIntegralSumLimit,LiftKf);
    FeedFoward feedFoward = new FeedFoward(pid);
    @Override
    public void runOpMode() throws InterruptedException
    {
        log = new Log("LogTest-1.0", true, telemetry);
        waitForStart();
        double state = 0;
        while (opModeIsActive())
        {
            double val = feedFoward.calculate(600, state, 1, Math.toRadians(45)).centerOffGravity(Math.toRadians(45), state);
            log.addData(val, state);
            log.update();
            telemetry.addData("Count", i);
            telemetry.update();
            i++;
            if (state < 600)
            {
                state++;
            }

        }
        log.close();
    }
}
