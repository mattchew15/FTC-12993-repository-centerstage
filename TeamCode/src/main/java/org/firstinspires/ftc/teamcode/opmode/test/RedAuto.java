package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.hardware.Print;
import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;
import org.firstinspires.ftc.teamcode.system.vision.RedTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.hardware.LoopTime;

@Autonomous
public class RedAuto extends LinearOpMode {
    RedTeamPropDetectorPipeline teamPropPipeline = new RedTeamPropDetectorPipeline();
    VisionHardware robot = new VisionHardware();
    Print print = new Print();
    LoopTime loopTime; // custom loop time class

    @Override
    public void runOpMode() {
        Globals.auto = true;
        Globals.redAuto = true;

        robot.init(hardwareMap);

        while(!isStarted()) {
            print.telemetryRedWebcam(telemetry);

            telemetry.update();
        }

        waitForStart();
        while(opModeIsActive()) {
            loopTime.updateLoopTime(telemetry);
            print.telemetryRedWebcam(telemetry);

            telemetry.update();
        }

        //place purple pixel at 'position'
    }
}
