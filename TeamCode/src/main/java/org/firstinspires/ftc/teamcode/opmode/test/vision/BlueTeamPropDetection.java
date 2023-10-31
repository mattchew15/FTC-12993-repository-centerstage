package org.firstinspires.ftc.teamcode.opmode.test.vision;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.BLUE_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.accessory.PrintCamera;
import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

@Autonomous(group = "test")
public class BlueTeamPropDetection extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final PrintCamera print = new PrintCamera();

    @Override
    public void runOpMode() {
        SetAuto.setBlueAuto();
        hardware.initWebcam(hardwareMap);

        BLUE_POSITION = hardware.getBluePosition();

        while(!isStarted()) {
            print.telemetryTeamProp();
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            print.telemetryTeamProp();
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
