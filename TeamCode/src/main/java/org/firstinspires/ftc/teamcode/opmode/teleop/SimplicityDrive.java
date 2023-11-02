package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.Sequences.Sequences;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpDown;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;


@TeleOp(name = "SimplicityDrive")
public class SimplicityDrive extends LinearOpMode {

    //Subsystems
    DriveBase driveBase = new DriveBase();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    //Accessories
    LoopTime loopTime = new LoopTime();
    ToggleUpDown intakeStackToggle = new ToggleUpDown(2); // 0,1,2 =  3 states

    ElapsedTime GlobalTimer;
    double sequenceTimer;



    enum OuttakeState { // could make these private?
        READY,
        INTAKE,
        INTAKE_EXTENDO,
        INTAKE_TO_TRANSFER,
        TRANSFER_START,
        TRANSFER_END,
        OUTTAKE_ADJUST,
        DEPOSIT,
        RETURN,
        MANUAL_ENCODER_RESET,
        IDLE
    }

    OuttakeState outtakeState;

    @Override
    public void runOpMode() {
        /*
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
         */

        StandardTrackingWheelLocalizer location = new StandardTrackingWheelLocalizer(hardwareMap,null,null); // set last 2 values to null?
        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        driveBase.initDrivebase(hardwareMap);
        outtakeState = OuttakeState.READY;


        waitForStart();
        if (opModeIsActive()) {

            GlobalTimer = new ElapsedTime(System.nanoTime());
            GlobalTimer.reset();
            sequenceTimer = GlobalTimer.milliseconds(); // sets up timer for caching
            intakeSubsystem.intakeHardwareSetup();
            outtakeSubsystem.hardwareSetup();
            driveBase.drivebaseSetup();

            while (opModeIsActive()) {

                outtakeSubsystem.outtakeReads(); // read at the start of the loop
                intakeSubsystem.intakeReads(); // could change both of these into one method in robothardware
                // can be condensed into the one class? - try ita
                loopTime.updateLoopTime(telemetry); // this may or may not work
                driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

                outtakeSequence();

                location.update();
                telemetry.update();
                //clears the cache at the end of the loop
                // PhotonCore.CONTROL_HUB.clearBulkCache();
            }
        }
    }

    public void outtakeSequence(){
        switch (outtakeState){
            case READY:
                outtakeSubsystem.pitchToInternalPID(SIXTY_DEGREE_TICKS,1); // internalPID uses less power draw - integral term is better
                outtakeSubsystem.liftToInternalPID(-2,1);
                outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.OPEN);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT,0); // need to create another method to point to backdrop so i don't need to pass through zero each time
                outtakeSubsystem.wristServoState(OuttakeSubsystem.WristServoState.READY);

                intakeSubsystem.intakeFlapServoState(IntakeSubsystem.IntakeFlapServoState.CLOSE);
                intakeSubsystem.intakeSlideInternalPID(0,1);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING);
                setIntakeArmHeight();

                if (gamepad1.right_bumper || gamepad2.right_bumper){
                    outtakeState = OuttakeState.INTAKE;
                }
                else if (gamepad1.left_bumper){
                    outtakeState = OuttakeState.INTAKE_EXTENDO;
                    sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                }

                break;

            case INTAKE:  // idk what this case does
                intakeSubsystem.intakeSpin(1);

                setIntakeArmHeight();

                if (gamepad2.left_bumper){ // or a sensor detects this
                    outtakeState = OuttakeState.TRANSFER_START;
                }
                break;

            case INTAKE_EXTENDO:  // idk what this case does
                intakeSubsystem.intakeSlideTo(INTAKE_SLIDE_EXTENDO_TELEOP, intakeSubsystem.intakeSlidePosition, 1);
                if (GlobalTimer.milliseconds() - sequenceTimer > 100){
                    intakeSubsystem.intakeSpin(1);
                }
                break;

            case INTAKE_TO_TRANSFER:  // idk what this case does
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                intakeSubsystem.intakeFlapServoState(IntakeSubsystem.IntakeFlapServoState.CLOSE);
                outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.OPEN);
                break;

            case TRANSFER_START:
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT, 0);
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.TRANSFER);
                outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.CLOSE);
                intakeSubsystem.intakeSpin(0.5);
                if (GlobalTimer.milliseconds() - sequenceTimer > 2000){
                    outtakeState = OuttakeState.;
                    sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                }
                break;
            case TRANSFER_END:
                intakeSubsystem.intakeFlapServoState(IntakeSubsystem.IntakeFlapServoState.OPEN);
                //outtakeSubsystem.wristServoState(OuttakeSubsystem.WristServoState.TRANSFER);
                if (GlobalTimer.milliseconds() - transferGrabTimer > 200){
                    transferGrabState = TransferGrabState.ARM_PRE_EXTEND;
                }
                break;
            case OUTTAKE_ADJUST:
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_UP);
                outtakeSubsystem.wristServoState(OuttakeSubsystem.WristServoState.SCORE);
                if (dropBtn){
                    outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.OPEN);
                }
                break;

            case DEPOSIT:  // idk what this case does
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                intakeSubsystem.intakeFlapServoState(IntakeSubsystem.IntakeFlapServoState.CLOSE);
                outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.OPEN);
                break;

            case RETURN:  // idk what this case does
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                intakeSubsystem.intakeFlapServoState(IntakeSubsystem.IntakeFlapServoState.CLOSE);
                outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.OPEN);

                intakeStackToggle.CycleState = 0; // sets the stack height to default
                break;

            case MANUAL_ENCODER_RESET:  // idk what this case does
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                intakeSubsystem.intakeFlapServoState(IntakeSubsystem.IntakeFlapServoState.CLOSE);
                outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.OPEN);
                break;
        }

        // this happens constantly throughout the loop
    }

    public void setIntakeArmHeight(){ // if seperating into classess pass the gamepad inputs into here
        intakeStackToggle.cycleToggleUpDown(gamepad2.right_bumper,gamepad2.left_bumper);
        if (intakeStackToggle.CycleState == 0){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
        } else if (intakeStackToggle.CycleState == 1){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE_STACK);
        } else if (intakeStackToggle.CycleState == 2){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP_STACK);
        }
    }

}


