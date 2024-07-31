package org.firstinspires.ftc.teamcode.opmode.test;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.ticksToInchesSlidesMotor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.accessory.PIDR;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
@Config
@TeleOp(name= "PIDEvaluation", group = "Test")
public class PIDEvalution extends LinearOpMode
{
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    public static double p = 0.009, i = 0.0001, d = 0.00002, f = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        LoopTime loopTime = new LoopTime(telemetry);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        outtakeSubsystem.initOuttake(hardwareMap);
        outtakeSubsystem.encodersReset();
        //outtakeSubsystem.encodersReset();
        outtakeSubsystem.hardwareSetup();
        outtakeSubsystem.cacheInitialPitchValue();
        int target = 0;
        boolean usePIDR = false;

        outtakeSubsystem.liftPIDR = new PIDR(p, i, d, f);
        waitForStart();

        while(opModeIsActive())
        {

            outtakeSubsystem.outtakeReads(false);
            if (gamepad1.a)
            {
                target = 1;
                usePIDR = false;

            }
            else if (gamepad1.x)
            {
                target = 14;
                usePIDR = true;
            }
            else if (gamepad1.y)
            {
                target = 7;
                usePIDR = true;
            }
            else if (gamepad1.b)
            {
                target = 1;
                usePIDR = true;
            }

            if(usePIDR)
            {
                outtakeSubsystem.liftToPIDR(target);
            }
            else
            {
                outtakeSubsystem.liftToInternalPID(target, 1);
            }
            outtakeSubsystem.pitchToInternalPID(36, 1);
            telemetry.addData("Target", outtakeSubsystem.liftTarget);
            telemetry.addData("LiftPosition", outtakeSubsystem.liftPosition);
            telemetry.addData("Power", outtakeSubsystem.LiftMotor.getPower());
            telemetry.addData("Error", outtakeSubsystem.liftPIDR.returnError());
            telemetry.addData("Percent error", Math.abs(outtakeSubsystem.liftTarget - outtakeSubsystem.liftPosition) / outtakeSubsystem.liftTarget * 100);
            telemetry.addData("Use PIDR", usePIDR);

            loopTime.updateLoopTime(telemetry);
            telemetry.update();
        }

    }
}
