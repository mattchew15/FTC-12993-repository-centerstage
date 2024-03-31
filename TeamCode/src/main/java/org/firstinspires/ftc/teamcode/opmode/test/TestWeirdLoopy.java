/*
package org.firstinspires.ftc.teamcode.opmode.test;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

import java.util.List;

@Photon
@TeleOp(name="TestLoopy", group = "Test")
public class TestWeirdLoopy extends LinearOpMode
{
    LoopTime loopTime = new LoopTime();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystemOptimised intakeSubsystem = new IntakeSubsystemOptimised();


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
            //intakeSubsystem.intakeReads(true);
            */
/*if (gamepad1.a && !prev)
            {
                target -= 1;

            }
            else if (gamepad1.y && !prev)
            {
                target += 1;

            }
            prev = gamepad1.a || gamepad1.y;
            outtakeSubsystem.liftToInternalPID(target, 1);
*//*

            if (gamepad1.a)
            {
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
            }
            if (gamepad1.b)
            {
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
            }

            //loopTime.delta();
            //telemetry.addData("Dt", 10000000 / loopTime.getDt());
            //telemetry.addData("Hz", loopTime.getHz());
            telemetry.addData("Target", target);
            telemetry.addData("Run target", outtakeSubsystem.prevLiftTarget);
            loopTime.updateLoopTime(telemetry);
            telemetry.update();
            for (LynxModule module : hubs) { // turns on bulk reads cannot  double read or it will call multiple bulkreads in the one thing
                module.clearBulkCache();
            }
        }
    }
}
*/
