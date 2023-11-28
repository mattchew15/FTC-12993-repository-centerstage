package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@Config
@TeleOp(group = "Tune")
public class ArmTuner extends LinearOpMode {
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();

    public static double OUTTAKE_ARM_POS = 0.5;
    @Override
    public void runOpMode() {

        outtakeSubsystem.initOuttake(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            outtakeSubsystem.OuttakeArmServoLeft.setPosition(OUTTAKE_ARM_POS);
            outtakeSubsystem.OuttakeArmServoRight.setPosition(OUTTAKE_ARM_POS);
        }
    }
}
