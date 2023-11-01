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
    double transferGrabTimer;

    //Sequences sequences = new Sequences(intakeSubsystem,outtakeSubsystem,GlobalTimer);

    enum TransferGrabState { // could make these private?
        READY,
        PIXEL_GRAB,
        FLAP_OPEN,
        ARM_PRE_EXTEND,
        IDLE
    }
    TransferGrabState transferGrabState;

    @Override
    public void runOpMode() {
        /*
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
         */

        //StandardTrackingWheelLocalizer location = new StandardTrackingWheelLocalizer(hardwareMap);
        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        driveBase.initDrivebase(hardwareMap);
        transferGrabState = TransferGrabState.IDLE;


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
                driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

                transferGrab(gamepad1.right_bumper);

                if(gamepad1.a){
                    transferGrabState = TransferGrabState.READY;
                }
                if (gamepad1.b){
                    outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.CLOSE);
                }
                if (gamepad1.y){
                    outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.OPEN);
                }

                intakeSubsystem.intakeSpin(gamepad1.right_trigger-gamepad1.left_trigger);

                //location.update();
                telemetry.update();
                //clears the cache at the end of the loop
                // PhotonCore.CONTROL_HUB.clearBulkCache();
            }
        }
    }

    public void transferGrab(boolean dropBtn){
        switch (transferGrabState){
            case READY:
                transferGrabTimer = GlobalTimer.milliseconds(); // resets timer
                transferGrabState = TransferGrabState.PIXEL_GRAB;
                break;
            case PIXEL_GRAB:
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT, 0);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.TRANSFER);
                outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.CLOSE);
                intakeSubsystem.intakeSpin(0.5);
                if (GlobalTimer.milliseconds() - transferGrabTimer > 2000){
                    transferGrabState = TransferGrabState.FLAP_OPEN;
                    transferGrabTimer = GlobalTimer.milliseconds(); // resets timer
                }
                break;
            case FLAP_OPEN:
                intakeSubsystem.intakeFlapServoState(IntakeSubsystem.IntakeFlapServoState.OPEN);
                //outtakeSubsystem.wristServoState(OuttakeSubsystem.WristServoState.TRANSFER);
                if (GlobalTimer.milliseconds() - transferGrabTimer > 200){
                    transferGrabState = TransferGrabState.ARM_PRE_EXTEND;
                }
                break;
            case ARM_PRE_EXTEND:
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_UP);
                outtakeSubsystem.wristServoState(OuttakeSubsystem.WristServoState.SCORE);
                if (dropBtn){
                    outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.OPEN);
                }
                break;

            case IDLE:
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                intakeSubsystem.intakeFlapServoState(IntakeSubsystem.IntakeFlapServoState.CLOSE);
                outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.OPEN);
                break;
        }
    }

}


