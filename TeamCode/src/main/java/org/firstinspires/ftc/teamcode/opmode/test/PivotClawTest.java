package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@Config
@TeleOp(group = "Tune")
public class PivotClawTest extends LinearOpMode {

    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();

    public static double
            PIVOT_POS = 0.535,
            CLAW_POS = 0.6;

    @Override
    public void runOpMode() {

        outtakeSubsystem.initOuttake(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            outtakeSubsystem.PivotServo.setPosition(PIVOT_POS);
            //outtakeSubsystem.ClawServo.setPosition(CLAW_POS);
        }
    }
}
