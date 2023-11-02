package org.firstinspires.ftc.teamcode.opmode.test;

// Old imports, some not needed
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;


@TeleOp(name = "PIDmotortest")
public class PIDMotorTest extends LinearOpMode {

    DriveBase driveBase = new DriveBase();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem(); // test to see if you get a null pointer error if you remove this - to test if the robothardware.setupHardware is inefficient
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    LoopTime loopTime; // might have to create new instance of class instead of just declaring it (who knows)

    // uses the ElapsedTime class from the SDK to create variable GlobalTimer
    ElapsedTime GlobalTimer;

    // random setup function that runs once start is pressed but before main loop
    // this setup should be put somewhere else


    @Override
    public void runOpMode() {

        // this is basically init, all setup, hardware classes etc get initialized here
        /*
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
         */

       // StandardTrackingWheelLocalizer location = new StandardTrackingWheelLocalizer(hardwareMap); // idk what other parameters are needed here
        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        driveBase.initDrivebase(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {

                GlobalTimer = new ElapsedTime(System.nanoTime());
                GlobalTimer.reset();
                intakeSubsystem.intakeHardwareSetup();
                outtakeSubsystem.hardwareSetup();
                driveBase.drivebaseSetup();

            while (opModeIsActive()) {

                outtakeSubsystem.outtakeReads(); // read at the start of the loop

                // can be condensed into the one class? - try ita
                loopTime.updateLoopTime(telemetry); // this may or may not work

                if(gamepad1.a){
                    outtakeSubsystem.liftTo(0, outtakeSubsystem.liftPosition, 1);
                }
                else if (gamepad1.b){
                    outtakeSubsystem.liftTo(100, outtakeSubsystem.liftPosition, 1);
                }

              //  location.update();
                telemetry.update();
                //clears the cache at the end of the loop
                // PhotonCore.CONTROL_HUB.clearBulkCache();
            }
        }
    }

}


