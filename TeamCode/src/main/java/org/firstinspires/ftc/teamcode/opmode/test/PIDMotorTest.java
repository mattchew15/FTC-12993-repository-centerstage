package org.firstinspires.ftc.teamcode.opmode.test;

// Old imports, some not needed
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.RobotHardware;


@TeleOp(name = "PIDmotortest")
public class PIDMotorTest extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem(); // cyclic dependency? could be an issue idk
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    double dt;
    double prev_time;

    // uses the ElapsedTime class from the SDK to create variable GlobalTimer
    ElapsedTime GlobalTimer;

    // random setup function that runs once start is pressed but before main loop
    // this setup should be put somewhere else
    private void Setup() {
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        outtakeSubsystem.hardwareSetup();
        intakeSubsystem.intakeHardwareSetup();
    }

    @Override
    public void runOpMode() {

        // this is basically init, all setup, hardware classes etc get initialized here
        /*
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
         */

        StandardTrackingWheelLocalizer location = new StandardTrackingWheelLocalizer(hardwareMap); // idk what other parameters are needed here
        robotHardware.initializeHardware(hardwareMap);


        waitForStart();
        if (opModeIsActive()) {

            Setup();

            while (opModeIsActive()) {

                outtakeSubsystem.outtakeReads(); // read at the start of the loop

                // can be condensed into the one class? - try it
                dt = System.currentTimeMillis() - prev_time;
                prev_time = System.currentTimeMillis();
                telemetry.addData("Loop Time", dt);

                if(gamepad1.a){
                    outtakeSubsystem.liftTo(0, outtakeSubsystem.liftPosition, 1);
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


