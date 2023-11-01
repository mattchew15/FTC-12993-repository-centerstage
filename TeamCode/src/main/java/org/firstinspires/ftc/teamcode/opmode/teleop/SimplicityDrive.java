package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.Sequences.Sequences;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;


@TeleOp(name = "SimplicityDrive")
public class SimplicityDrive extends LinearOpMode {

    DriveBase driveBase = new DriveBase();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    LoopTime loopTime = new LoopTime();
    ElapsedTime GlobalTimer;

    Sequences sequences = new Sequences(intakeSubsystem,outtakeSubsystem,GlobalTimer);


    @Override
    public void runOpMode() {
        /*
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
         */

        StandardTrackingWheelLocalizer location = new StandardTrackingWheelLocalizer(hardwareMap);
        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        driveBase.initDrivebase(hardwareMap);
        sequences.setupSequences();


        waitForStart();
        if (opModeIsActive()) {

            GlobalTimer = new ElapsedTime(System.nanoTime());
            GlobalTimer.reset();
            intakeSubsystem.intakeHardwareSetup();
            outtakeSubsystem.hardwareSetup();
            driveBase.drivebaseSetup();

            while (opModeIsActive()) {

                outtakeSubsystem.outtakeReads(); // read at the start of the loop
                intakeSubsystem.intakeReads(); // could change both of these into one method in robothardware
                // can be condensed into the one class? - try ita
                loopTime.updateLoopTime(telemetry); // this may or may not work
                driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad2.left_stick_x);

                sequences.transferGrab(gamepad1.right_bumper);

                if(gamepad1.a){
                    sequences.startTransfer();
                }

                location.update();
                telemetry.update();
                //clears the cache at the end of the loop
                // PhotonCore.CONTROL_HUB.clearBulkCache();
            }
        }
    }

}


