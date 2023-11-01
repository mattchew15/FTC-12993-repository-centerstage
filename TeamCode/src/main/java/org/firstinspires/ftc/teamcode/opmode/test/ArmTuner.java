package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;

@Config
@TeleOp
public class ArmTuner extends LinearOpMode {
    RobotHardware robotHardware = new RobotHardware();

    public static double OUTTAKE_ARM_POS = 0.5;
    @Override
    public void runOpMode() {

        robotHardware.initializeHardware(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            robotHardware.OuttakeArmServoLeft.setPosition(OUTTAKE_ARM_POS);
        }
    }
}
// verti
// cal up
// 0.31 right
//