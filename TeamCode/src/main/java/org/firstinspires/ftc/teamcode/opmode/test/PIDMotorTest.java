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
    double intakeClipTimer;
    private boolean prev;

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

            int target = 0;
            while (opModeIsActive()) {


                outtakeSubsystem.outtakeReads(false); // read at the start of the loop
                //intakeSubsystem.intakeReads(true);

                // can be condensed into the one class? - try ita
                loopTime.updateLoopTime(telemetry); // this may or may not work
                outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
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

                else if (gamepad1.x){
                    //outtakeSubsystem.liftTo(6,outtakeSubsystem.liftPosition,1);
                    outtakeSubsystem.liftToInternalPID(0, 1);
                }else if (gamepad1.y){
                    //outtakeSubsystem.liftTo(12,outtakeSubsystem.liftPosition,13);
                    outtakeSubsystem.liftToInternalPID(5, 1);
                }




                //outtakeSubsystem.liftTo(0, outtakeSubsystem.liftPosition, 1);
/*
                else if (gamepad1.dpad_left){
                    outtakeSubsystem.pitchToFTCLib(36);
                }else if (gamepad1.dpad_right){
                    outtakeSubsystem.pitchToFTCLib(30);
                }

                else if (gamepad1.dpad_down){
                    outtakeSubsystem.pitchToInternalPID(36,1);
                }else if (gamepad1.dpad_up){
                    outtakeSubsystem.pitchToInternalPID(30,1);
                }

 */



                if (gamepad1.dpad_left){
                    target = 0;
                }else if (gamepad1.dpad_right){
                    target = 3;
                }
                else if (gamepad1.dpad_up && !prev)
                {
                    target = 7;
                }
                else if (gamepad1.dpad_down && !prev)
                {
                    target = 12;
                }







                //intakeSubsystem.IntakeSlideMotor.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
                telemetry.addData("intake slide target", intakeSubsystem.intakeSlideTarget);
                telemetry.addData("IntakeSlidePosition", intakeSubsystem.intakeSlidePosition);
                driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                telemetry.addData("intake slide target reached", intakeSubsystem.intakeSlideTargetReached());
                telemetry.addData("lift Target Reached", outtakeSubsystem.liftTargetReached());
                telemetry.addData("Pitch target reached", outtakeSubsystem.liftTargetReached());

                telemetry.addData("Distance sensor value", outtakeSubsystem.outtakeDistanceSensorValue);
                telemetry.addData("LiftTarget", outtakeSubsystem.liftTarget);
                telemetry.addData("LiftPosition",ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition));
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
          //      telemetry.addData("Pitch interal encoder position", outtakeSubsystem.pitchPosition);
                telemetry.addData("internal pitch target", outtakeSubsystem.pitchTarget);
                telemetry.addData("Initial Cached Position Pitch", outtakeSubsystem.pitchTicksInitialOffset);
               // telemetry.addData("Raw Axon Encoder Reading", outtakeSubsystem.getPitchEncoderPos());
               // telemetry.addData("Degrees internal encoder", ticksToDegreePitchMotor(outtakeSubsystem.pitchPosition));
               // telemetry.addData("Degrees Axon encoder", outtakeSubsystem.getPitchEncoderPos());



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
    public void intakeClipHoldorNotHold(int slideToPosition){
        if (intakeSubsystem.intakeSlidePosition < 3) {
            if (GlobalTimer.milliseconds() - intakeClipTimer > 70){
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING); // turn the intake slide pid running to pos off to save battery draw
            }
            intakeSubsystem.intakeSlideMotorRawControl(0);
            telemetry.addLine("We are HOLDING STUFF IN");
        } else {
            intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // this might break something when as the intake slides won't go in, but stops jittering
            intakeSubsystem.intakeSlideInternalPID(slideToPosition,1);
            intakeClipTimer = GlobalTimer.milliseconds();
            telemetry.addLine("We are trying to slide our slides in");

        }
    }

}


