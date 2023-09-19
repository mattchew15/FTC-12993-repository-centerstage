package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.hardware.Print;
import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.system.hardware.VisionHardware;
import org.firstinspires.ftc.teamcode.system.vision.BlueTeamPropDetectorPipeline;
import org.firstinspires.ftc.teamcode.system.hardware.LoopTime;

@Autonomous
public class BlueAuto extends LinearOpMode {
    BlueTeamPropDetectorPipeline teamPropPipeline = new BlueTeamPropDetectorPipeline();
    VisionHardware robot = new VisionHardware();
    Print print = new Print();
    LoopTime loopTime; // custom loop time class

    @Override
    public void runOpMode() {
        Globals.auto = true;
        Globals.blueAuto = true;

        robot.init(hardwareMap);

        while(!isStarted()) {
            print.telemetryAprilTag(telemetry, robot.getApriltag());
            print.telemetryBlueWebcam(telemetry);

            telemetry.update();
        }

        waitForStart();
        while(opModeIsActive()) {
            print.telemetryAprilTag(telemetry, robot.getApriltag());
            print.telemetryBlueWebcam(telemetry);
            loopTime.updateLoopTime(telemetry);

            telemetry.update();
        }

        //place purple pixel at 'position'
    }
}
