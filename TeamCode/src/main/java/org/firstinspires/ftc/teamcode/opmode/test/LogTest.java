package org.firstinspires.ftc.teamcode.opmode.test;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.mathCaching;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.numCycles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.accessory.Log;
import org.firstinspires.ftc.teamcode.system.accessory.PID;
import org.firstinspires.ftc.teamcode.system.accessory.FeedFoward;
import org.firstinspires.ftc.teamcode.system.accessory.math.MathResult;

@Disabled
@TeleOp(name = "LOG  ")
public class LogTest extends LinearOpMode
{
    Log log;
    double i;
    private static double LiftKp = 0.015, LiftKi = 0.0001, LiftKd = 0.00006, LiftIntegralSumLimit = 10, LiftKf = 0;
    PID pid = new PID(LiftKp,LiftKi,LiftKd,LiftIntegralSumLimit,LiftKf);
    MathResult result = new MathResult(0, 0);
    FeedFoward feedFoward = new FeedFoward(pid);
    @Override
    public void runOpMode() throws InterruptedException
    {
        log = new Log("LogTest-2.0", true, telemetry);
        waitForStart();
        double state = 0;
        while (opModeIsActive() && !isStopRequested())
        {
            /*
            double val = feedFoward.calculate(600, state, 1, Math.toRadians(45)).centerOffGravity(Math.toRadians(45), state);
            double ff = feedFoward.getCurrentFeedForward();
            double pid = feedFoward.calculate(600, state, 1, Math.toRadians(45)).getPow();

            log.addData(val, ff, pid, state);
            log.update();
            telemetry.addData("Count", i);
            //telemetry.addData("Fast update", true);
            telemetry.update();
            i++;
            if (state < 700)
            {
                state++;
            }
            else
            {
                stop();
            }

             */

            result = mathCaching(Math::cos, 0, result.getPrevInput(), result.getResult());
            result = mathCaching(x -> (Math.cos(x)), 2, result.getPrevInput(), result.getResult());

        }
        log.close();
    }


}
