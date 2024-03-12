package org.firstinspires.ftc.teamcode.opmode.test;

// Old imports, some not needed
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import java.util.List;

@Config
@TeleOp(name = "PIDmotortest")
public class PIDMotorTest extends LinearOpMode {


    //DcMotorEx IntakeSlideMotor;
    DriveBase driveBase = new DriveBase();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem(); // test to see if you get a null pointer error if you remove this - to test if the robothardware.setupHardware is inefficient
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    LoopTime loopTime = new LoopTime(); // might have to create new instance of class instead of just declaring it (who knows)

    // uses the ElapsedTime class from the SDK to create variable GlobalTimer
    ElapsedTime GlobalTimer;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    // random setup function that runs once start is pressed but before main loop
    // this setup should be put somewhere else

    private List<LynxModule> hubs;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // this is basically init, all setup, hardware classes etc get initialized here
        /*
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
         */

       // StandardTrackingWheelLocalizer location = new StandardTrackingWheelLocalizer(hardwareMap); // idk what other parameters are needed here
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub: hubs)
        {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.clearBulkCache();
        }
        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        driveBase.initDrivebase(hardwareMap);

       // IntakeSlideMotor = hardwareMap.get(DcMotorEx.class, "IntakeSlideMotor");
        waitForStart();
        if (opModeIsActive()) {

                GlobalTimer = new ElapsedTime(System.nanoTime());
                GlobalTimer.reset();
               // intakeSubsystem.intakeHardwareSetup();
                outtakeSubsystem.hardwareSetup();
                driveBase.drivebaseSetup();
                outtakeSubsystem.encodersReset();
                intakeSubsystem.intakeSlideMotorEncodersReset();
                outtakeSubsystem.cacheInitialPitchValue();

            while (opModeIsActive()) {

                outtakeSubsystem.outtakeReads(false); // read at the start of the loop
                intakeSubsystem.intakeReads(false);

                // can be condensed into the one class? - try ita
                loopTime.updateLoopTime(telemetry); // this may or may not work

                if(gamepad1.a){
                    outtakeSubsystem.liftTo(0, outtakeSubsystem.liftPosition, 1);
                }
                else if (gamepad1.b){
                    outtakeSubsystem.liftTo(6, outtakeSubsystem.liftPosition, 1);
                }


                //driveBase.motorDirectionTest(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_y,gamepad1.right_stick_x);
/*
                else if (gamepad1.y) {
                    outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
                } else if (gamepad1.x){
                    outtakeSubsystem.pitchToInternalPID(PITCH_LOW_DEGREE_TICKS,1);
                }
 */

                else if (gamepad1.dpad_down){
                    //outtakeSubsystem.liftTo(6,outtakeSubsystem.liftPosition,1);
                    outtakeSubsystem.liftToInternalPID(-5,1);
                }else if (gamepad1.dpad_up){
                    //outtakeSubsystem.liftTo(12,outtakeSubsystem.liftPosition,13);
                    outtakeSubsystem.liftToInternalPID(0,1);
                }



                //outtakeSubsystem.liftTo(0, outtakeSubsystem.liftPosition, 1);

                else if (gamepad1.dpad_left){
                   intakeSubsystem.intakeSlideInternalPID(-10,1);
                }else if (gamepad1.dpad_right){
                    intakeSubsystem.intakeSlideInternalPID(200,1);
                }


                else if (gamepad1.dpad_up){
                    intakeSubsystem.intakeSlideTo(0,intakeSubsystem.intakeSlidePosition,1);
                }else if (gamepad1.dpad_down){
                    intakeSubsystem.intakeSlideTo(900,intakeSubsystem.intakeSlidePosition,1);
                }




                //intakeSubsystem.IntakeSlideMotor.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

                driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                telemetry.addData("IntakeSlidePosition", intakeSubsystem.intakeSlidePosition);
                telemetry.addData("Distance sensor value", outtakeSubsystem.outtakeDistanceSensorValue);
                telemetry.addData("LiftPosition",outtakeSubsystem.liftPosition);
                telemetry.addData("Lift Position Raw", outtakeSubsystem.liftPosition);
                telemetry.addData("pitch raw position",outtakeSubsystem.pitchPosition);
                telemetry.addLine("");
                telemetry.addData("Colour Sensor front", intakeSubsystem.frontColourSensorValue);
                telemetry.addData("Colour Sensor back", intakeSubsystem.backColourSensorValue);
                telemetry.addData("IntakeMotor Current", intakeSubsystem.intakeCurrent);
                telemetry.addData("IntakeChuteArmPosition", intakeSubsystem.intakeChuteArmPosition);
                telemetry.addLine("");
                telemetry.addData("LeftLimitSwitch", intakeSubsystem.leftArmLimitSwitchValue);
                telemetry.addData("RightLimitSwitch", intakeSubsystem.rightArmLimitSwitchValue);
                telemetry.addData("Chute Limit Switch", intakeSubsystem.chuteDetectorLimitSwitchValue);
                telemetry.addLine("");
                telemetry.addLine("");
                telemetry.addLine("");

                telemetry.addData("PitchPosition", outtakeSubsystem.pitchEncoderPosition);
               // telemetry.addData("Raw Axon Encoder Reading", outtakeSubsystem.getPitchEncoderPos());



                // this is the pitch position servo kinda difficult

                outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition, telemetry);

                //location.update();
                telemetry.update();
                //clears the cache at the end of the loop
                // PhotonCore.CONTROL_HUB.clearBulkCache()
                clearCache();
            }
        }
    }

    public void clearCache()
    {
        for (LynxModule hub: hubs)
        {
            hub.clearBulkCache();
        }
    }

}


