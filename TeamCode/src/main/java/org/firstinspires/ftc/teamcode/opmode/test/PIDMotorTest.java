package org.firstinspires.ftc.teamcode.opmode.test;

// Old imports, some not needed
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;


@TeleOp(name = "PIDmotortest")
public class PIDMotorTest extends LinearOpMode {


    //DcMotorEx IntakeSlideMotor;
    DriveBase driveBase = new DriveBase();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem(); // test to see if you get a null pointer error if you remove this - to test if the robothardware.setupHardware is inefficient
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    LoopTime loopTime = new LoopTime(); // might have to create new instance of class instead of just declaring it (who knows)

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
       // IntakeSlideMotor = hardwareMap.get(DcMotorEx.class, "IntakeSlideMotor");
        waitForStart();
        if (opModeIsActive()) {

                GlobalTimer = new ElapsedTime(System.nanoTime());
                GlobalTimer.reset();
                intakeSubsystem.intakeHardwareSetup();
                outtakeSubsystem.hardwareSetup();
                driveBase.drivebaseSetup();
                outtakeSubsystem.encodersReset();
                intakeSubsystem.intakeSlideMotorEncodersReset(); // makesure its dcEX not dcmotor :(((


            while (opModeIsActive()) {

                outtakeSubsystem.outtakeReads(); // read at the start of the loop
                intakeSubsystem.intakeReads();

                // can be condensed into the one class? - try ita
                loopTime.updateLoopTime(telemetry); // this may or may not work

                if(gamepad1.a){
                    outtakeSubsystem.liftTo(0, outtakeSubsystem.liftPosition, 1);
                }
                else if (gamepad1.b){
                    outtakeSubsystem.liftTo(100, outtakeSubsystem.liftPosition, 1);
                } else if (gamepad1.y) {
                    outtakeSubsystem.pitchToInternalPID(0,0.5);
                } else if (gamepad1.x){
                    outtakeSubsystem.pitchToInternalPID(10,0.5);
                }

                else if (gamepad1.dpad_down){
                    outtakeSubsystem.liftToInternalPID(100,0.5);
                }else if (gamepad1.dpad_up){
                    outtakeSubsystem.liftToInternalPID(0,0.5);
                }

                else if (gamepad1.dpad_left){
                   intakeSubsystem.intakeSlideInternalPID(0,0.7);
                }else if (gamepad1.dpad_right){
                    intakeSubsystem.intakeSlideInternalPID(100,0.7);
                }
                //intakeSubsystem.IntakeSlideMotor.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
                driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

                telemetry.addData("IntakeSlidePosition", intakeSubsystem.intakeSlidePosition);
                telemetry.addData("LiftPosition", outtakeSubsystem.liftPosition);
                telemetry.addData("PitchPosition", outtakeSubsystem.pitchPosition);

                //location.update();
                telemetry.update();
                //clears the cache at the end of the loop
                // PhotonCore.CONTROL_HUB.clearBulkCache();
            }
        }
    }

}


