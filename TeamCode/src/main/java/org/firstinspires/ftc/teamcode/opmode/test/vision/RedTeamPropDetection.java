package org.firstinspires.ftc.teamcode.opmode.test.vision;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RED_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.accessory.Print;
import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.system.base.SetAuto;

@Autonomous(group = "test")
public class RedTeamPropDetection extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Print print = new Print();

    @Override
    public void runOpMode() {
        SetAuto.setRedAuto();
        hardware.initWebcam(hardwareMap);

        RED_POSITION = hardware.getRedPosition();

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
