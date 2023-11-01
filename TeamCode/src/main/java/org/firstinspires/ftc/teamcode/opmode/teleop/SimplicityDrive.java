package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.onbotjava.EditorSettings;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.Sequences.TeleopSequence;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;


@TeleOp(name = "SimplicityDrive")
public class SimplicityDrive extends LinearOpMode {


    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    RobotHardware robotHardware = new RobotHardware();
    LoopTime loopTime;
    TeleopSequence teleopSequence;

    ElapsedTime GlobalTimer;


    @Override
    public void runOpMode() {
        /*
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
         */

        StandardTrackingWheelLocalizer location = new StandardTrackingWheelLocalizer(hardwareMap);
        robotHardware.initializeHardware(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {

            GlobalTimer = new ElapsedTime(System.nanoTime());
            GlobalTimer.reset();
            robotHardware.setupHardware(); // this uses methods from the outtake and intake subsystem - are we creating multiple instances of the same class???

            while (opModeIsActive()) {

                outtakeSubsystem.outtakeReads(); // read at the start of the loop
                intakeSubsystem.intakeReads(); // could change both of these into one method in robothardware
                // can be condensed into the one class? - try ita
                loopTime.updateLoopTime(telemetry); // this may or may not work

                if(gamepad1.a){
                    teleopSequence.things();
                }
                else if (gamepad1.b){
                    outtakeSubsystem.liftTo(100, outtakeSubsystem.liftPosition, 1);
                }

                location.update();
                telemetry.update();
                //clears the cache at the end of the loop
                // PhotonCore.CONTROL_HUB.clearBulkCache();
            }
        }
    }

}


