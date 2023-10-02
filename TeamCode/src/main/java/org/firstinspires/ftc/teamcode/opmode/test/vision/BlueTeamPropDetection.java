package org.firstinspires.ftc.teamcode.opmode.test.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.accessory.Print;
import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

@Autonomous(group = "test")
public class BlueTeamPropDetection extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Print print = new Print();

    @Override
    public void runOpMode() {
        SetAuto.setBlueAuto();
        hardware.initWebcam(hardwareMap);

        while(!isStarted()) {
            print.telemetryBlueWebcam(hardware.getBluePosition());
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            print.telemetryBlueWebcam(hardware.getBluePosition());
            telemetry.update();
        }

        //place purple pixel at 'position'

    }
}
