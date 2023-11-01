package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;

@Config
@TeleOp
public class HardwareTune extends LinearOpMode {
    RobotHardware robotHardware;

    @Override
    public void runOpMode() {

        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
