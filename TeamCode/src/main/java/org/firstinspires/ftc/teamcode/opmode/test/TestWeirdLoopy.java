package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="TestLoopy", group = "Test")
public class TestWeirdLoopy extends LinearOpMode
{
    LoopTime loopTime = new LoopTime();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();


    @Override
    public void runOpMode() throws InterruptedException
    {
        outtakeSubsystem.initOuttake(hardwareMap);
        outtakeSubsystem.hardwareSetup();
        intakeSubsystem.initIntake(hardwareMap);
        intakeSubsystem.intakeHardwareSetup();
        double target = 2;
        boolean prev = false;
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : hubs) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            module.clearBulkCache();
        }
        waitForStart();


        while(opModeIsActive())
        {
            outtakeSubsystem.outtakeReads(false);
            intakeSubsystem.intakeReads(false);
            if (gamepad1.a && !prev)
            {
                target -= 1;

            }
            else if (gamepad1.y && !prev)
            {
                target += 1;

            }
            prev = gamepad1.a || gamepad1.y;
            outtakeSubsystem.liftToInternalPID(target, 1);


            //loopTime.delta();
            //telemetry.addData("Dt", 10000000 / loopTime.getDt());
            //telemetry.addData("Hz", loopTime.getHz());
            loopTime.updateLoopTime(telemetry);
            telemetry.update();
            for (LynxModule module : hubs) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
                module.clearBulkCache();
            }
        }
    }
}
