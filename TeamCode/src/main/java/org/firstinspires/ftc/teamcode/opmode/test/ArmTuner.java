package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;

@Config
@TeleOp
public class ArmTuner extends LinearOpMode {
    RobotHardware robotHardware;

    public static double
            OUTTAKE_ARM_LEFT_POS = 0.5,
            OUTTAKE_ARM_RIGHT_POS = 0.5;

    @Override
    public void runOpMode() {

        robotHardware.initializeHardware(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a){
                robotHardware.OuttakeArmServoLeft.setPosition(OUTTAKE_ARM_LEFT_POS);
            }
            else if (gamepad1.b){
                robotHardware.OuttakeArmServoRight.setPosition(OUTTAKE_ARM_RIGHT_POS);
            }
        }
    }
}
