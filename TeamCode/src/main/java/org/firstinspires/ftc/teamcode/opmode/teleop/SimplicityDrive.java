package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.accessory.Toggle;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpDown;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import android.graphics.Path;


@TeleOp(name = "SimplicityDrive")
public class SimplicityDrive extends LinearOpMode {

    //Subsystems
    DriveBase driveBase = new DriveBase();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    //Accessories
    LoopTime loopTime = new LoopTime();
    ToggleUpDown intakeStackToggle = new ToggleUpDown(4, 0); // starting cycle state = 0
    ToggleUpDown miniTurretPositionToggle = new ToggleUpDown(2, 1); // one starts at the centre
    Toggle pivotFlipToggle = new Toggle();

    ElapsedTime GlobalTimer;
    double sequenceTimer;
    int liftTarget;
    int pitchTarget;
    boolean pitching;

    enum OuttakeState { // could make these private?
        READY,
        INTAKE_EXTENDO,
        INTAKE,
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
            pitching = false; // are we pitching initially? probably

            while (opModeIsActive()) {

                outtakeSubsystem.outtakeReads(); // read at the start of the loop
                intakeSubsystem.intakeReads(); // could change both of these into one method in robothardware
                // can be condensed into the one class? - try ita
                loopTime.updateLoopTime(telemetry); // this may or may not work
                driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

                outtakeSequence();

                location.update();
                telemetry.addData("STATE", outtakeState);
                telemetry.addData("IntakeSlidePosition", intakeSubsystem.intakeSlidePosition);
                telemetry.addData("LiftPosition", outtakeSubsystem.liftPosition);
                telemetry.addData("PitchPosition", outtakeSubsystem.pitchPosition);
                telemetry.addLine("");
                telemetry.addData("IntakeSlideTargetReached", intakeSubsystem.intakeSlideTargetReached());

                telemetry.update();
                //clears the cache at the end of the loop
                // PhotonCore.CONTROL_HUB.clearBulkCache();
            }
        }
    }

    public void outtakeSequence(){
        switch (outtakeState){
            case READY:

                readyOuttake();
                liftTarget = 0; // this isn't the lift target in this state but rather for when we are fine adjusting lift
                pitchTarget = SIXTY_DEGREE_TICKS;

                intakeClipHoldorNotHold(-3);
                intakeSubsystem.intakeFlapServoState(IntakeSubsystem.IntakeFlapServoState.CLOSE);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING);
                setIntakeArmHeight();
                pivotFlipToggle.ToggleMode(gamepad2.dpad_up); // this could go on states but it may as well be here cos it is run all the time

                if (gamepad1.right_bumper || gamepad2.b) {
                    outtakeState = OuttakeState.INTAKE;
                }
                else if (gamepad1.left_bumper) {
                    outtakeState = OuttakeState.INTAKE_EXTENDO;
                    sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                }

                break;

            case INTAKE:
                intakeSubsystem.intakeSpin(1);

                setIntakeArmHeight();

                if (gamepad2.right_trigger > 0.2) { // or a sensor detects this
                    outtakeState = OuttakeState.INTAKE_TO_TRANSFER; //reverses intake
                    sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                }
                break;

            case INTAKE_EXTENDO:
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                setIntakeArmHeight();
                if (GlobalTimer.milliseconds() - sequenceTimer > 60) { // power draw reasons
                    intakeSubsystem.intakeSlideTo(INTAKE_SLIDE_EXTENDO_TELEOP, intakeSubsystem.intakeSlidePosition, 1);
                    if (GlobalTimer.milliseconds() - sequenceTimer > 200){
                        intakeSubsystem.intakeSpin(1);
                        if (gamepad2.right_trigger > 0.2) { // or a sensor detects this
                            outtakeState = OuttakeState.INTAKE_TO_TRANSFER;
                            sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                        }
                    }
                }
                break;

            case INTAKE_TO_TRANSFER:

                intakeSubsystem.intakeSpin(-0.7); // helps the slides go in cos of bottom roller?
                intakeClipHoldorNotHold(-3);

                if (intakeSubsystem.intakeSlidePosition < 2 && GlobalTimer.milliseconds() - sequenceTimer > 150){ // could tune this more finely
                    outtakeState = OuttakeState.TRANSFER_START;
                    sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                }
                break;

            case TRANSFER_START:
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.TRANSFER);

                if (GlobalTimer.milliseconds() - sequenceTimer > 50) {
                    outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.CLOSE);
                    if (GlobalTimer.milliseconds() - sequenceTimer > 300){
                        outtakeState = OuttakeState.TRANSFER_END;
                        sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                        intakeSubsystem.intakeFlapServoState(IntakeSubsystem.IntakeFlapServoState.OPEN);
                    }
                }
                break;

            case TRANSFER_END:
                outtakeSubsystem.wristServoState(OuttakeSubsystem.WristServoState.TRANSFER);
                if (GlobalTimer.milliseconds() - sequenceTimer > 30){
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.PRE_EXTEND);
                    if ((liftTarget != 0 || pitchTarget!=0) && GlobalTimer.milliseconds() - sequenceTimer > 150) { // basically if we have selected a scoring position
                        outtakeState = OuttakeState.OUTTAKE_ADJUST;
                    }
                }
                break;

            case OUTTAKE_ADJUST:
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_UP);
                outtakeSubsystem.wristServoState(OuttakeSubsystem.WristServoState.SCORE);

                outtakeSubsystem.liftTo(liftTarget, outtakeSubsystem.liftPosition,1);

                if (outtakeSubsystem.liftPosition > LIFT_HITS_WHILE_PITCHING_THRESHOLD) { // so shit doesn't hit the thing when pitching
                    outtakeSubsystem.pitchToInternalPID(pitchTarget,1);
                }

                pivotAdjust();
                miniTurretAdjust();
                miniTurretPositionToggle.cycleToggleUpDown(gamepad1LeftTrigger(),gamepad1RightTrigger());

                if (gamepad1.right_bumper) {
                    outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.OPEN);
                    sequenceTimer = GlobalTimer.milliseconds();// reset timer
                }
                break;

            case DEPOSIT:  // idk what this case does
                if (GlobalTimer.milliseconds() - sequenceTimer > 300){ // 300 ms wait after dropping
                    outtakeState = OuttakeState.RETURN;
                    sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                }
                break;

            case RETURN:  // idk what this case does
                intakeSubsystem.intakeFlapServoState(IntakeSubsystem.IntakeFlapServoState.OPEN);
                outtakeSubsystem.pitchToInternalPID(SIXTY_DEGREE_TICKS,1);
                intakeSubsystem.intakeSlideInternalPID(-3,1);
                if (GlobalTimer.milliseconds() - sequenceTimer > 50 && intakeSubsystem.intakeSlideTargetReached()){ // make sure the flap is open
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                    outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.OPEN);
                }
                if (!outtakeSubsystem.pitchTargetReached()){
                    outtakeSubsystem.liftTo(LIFT_HITS_WHILE_PITCHING_THRESHOLD, outtakeSubsystem.liftPosition,1);
                } else {
                    outtakeSubsystem.liftToInternalPID(0,1);
                }

                if (outtakeSubsystem.liftPosition < 5){
                        outtakeState = OuttakeState.READY; //:)
                }

                intakeStackToggle.CycleState = 0; // sets the stack height to default
                break;

            case MANUAL_ENCODER_RESET:  // idk what this case does
                outtakeSubsystem.liftMotorRawControl(gamepad2.right_stick_y);
                outtakeSubsystem.pitchMotorRawControl(gamepad2.left_stick_y);
                intakeSubsystem.intakeSlideMotorRawControl(gamepad2.left_trigger-gamepad2.right_trigger);

                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT); // need to create another method to point to backdrop so i don't need to pass through zero each time
                outtakeSubsystem.wristServoState(OuttakeSubsystem.WristServoState.READY);
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
                outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.OPEN);

                intakeSubsystem.intakeFlapServoState(IntakeSubsystem.IntakeFlapServoState.CLOSE);

                if (gamepad2.right_bumper){
                    outtakeSubsystem.encodersReset();
                    intakeSubsystem.intakeSlideMotorEncodersReset();
                    outtakeState = OuttakeState.READY;
                }
                break;
        }

        // this happens constantly throughout the loop
        if ((gamepad2.b || gamepad1.b) && (outtakeState != OuttakeState.READY)){
            outtakeState = OuttakeState.RETURN; // if b is pressed at any state then return to ready
            pivotFlipToggle.ToggleMode = false; // returning should put thing like this to default
            intakeStackToggle.CycleState = 0;
            miniTurretPositionToggle.CycleState = 1; // reset turret to straight
        }
        if (gamepad2.options){ // shouldn't need a toggle function for this
            outtakeState = OuttakeState.MANUAL_ENCODER_RESET;
        }
        liftPositionChange();
        pitchingOrNot();
    }

    public void setIntakeArmHeight(){ // Change this shit that lotus messed up
        intakeStackToggle.cycleToggleUpDown(gamepad2.right_bumper,gamepad2.left_bumper);
        switch (intakeStackToggle.CycleState) {
            case 0:
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.HEIGHT_1);
                break;
            case 1:
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.HEIGHT_2);
                break;
            case 2:
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.HEIGHT_3);
                break;
            case 3:
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.HEIGHT_4);
                break;
            case 4:
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.HEIGHT_5);
                break;
        }
    }

    public void intakeClipHoldorNotHold(int slideToPosition){
        if (intakeSubsystem.intakeSlidePosition > -5) {
            intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING); // turn the intake slide pid running to pos off to save battery draw
            intakeSubsystem.intakeSlideMotorRawControl(0);
        } else {
            intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // this might break something when as the intake slides won't go in, but stops jittering
            intakeSubsystem.intakeSlideInternalPID(slideToPosition,1);
        }
    }

    public void readyOuttake(){
        outtakeSubsystem.pitchToInternalPID(SIXTY_DEGREE_TICKS,1); // internalPID uses less power draw - integral term is better
        outtakeSubsystem.liftToInternalPID(0,1);
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT); // need to create another method to point to backdrop so i don't need to pass through zero each time
        outtakeSubsystem.wristServoState(OuttakeSubsystem.WristServoState.READY);
        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
        outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.OPEN);
    }
    public void liftPositionChange(){ // if gamepad inputs don't work in this class then pass them through as parameters in the function
        if (gamepad2.y || gamepad1.y){
            liftTarget = LIFT_HIGH_POSITION_TICKS;
            pitchTarget = SIXTY_DEGREE_TICKS;
        } else if (gamepad2.x || gamepad1.x){
            if (pitching){
                liftTarget = LIFT_MEDIUM_POSITION_PITCHING_TICKS;
                pitchTarget = PITCH_MID_DEGREE_TICKS;
            } else if (!pitching){
                pitchTarget = SIXTY_DEGREE_TICKS;
                liftTarget = LIFT_MEDIUM_POSITION_TICKS;
            }
        } else if (gamepad2.a || gamepad1.a){
            if (pitching){
                liftTarget = LIFT_LOW_POSITION_PITCHING_TICKS;
                pitchTarget = PITCH_LOW_DEGREE_TICKS;
            } else if (!pitching){
                liftTarget = LIFT_LOW_POSITION_TICKS;
                pitchTarget = SIXTY_DEGREE_TICKS;
            }
        }
    }

    public void pivotAdjust(){
        if (gamepad2.left_trigger > 0.2) {
            if (pivotFlipToggle.ToggleMode){
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_LEFT_FLIPPED);
            } else {
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_LEFT);
            }
        } else if (gamepad2.right_trigger > 0.2){
            if (pivotFlipToggle.ToggleMode){
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_RIGHT_FLIPPED);
            } else {
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_RIGHT);
            }
        } else if (gamepad2.dpad_left){
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
        } else if (gamepad2.dpad_right){
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
        }
    }

    void miniTurretAdjust(){
        if (miniTurretPositionToggle.CycleState == 0){
            outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.DIAGONAL_LEFT);
        } else if (miniTurretPositionToggle.CycleState == 1) {
            outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
        } else if (miniTurretPositionToggle.CycleState == 2) {
            outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.DIAGONAL_RIGHT);
        }
    }

    public void pitchingOrNot(){
        if (gamepad2.right_stick_button){
            pitching = false;
        } else if (gamepad2.left_stick_button){
            pitching = true;
        }
    }

    boolean gamepad1RightTrigger(){
        if (gamepad1.right_trigger > GAMEPAD_TRIGGER_THRESHOLD){
            return true;
        } else {
            return false;
        }
    }
    boolean gamepad1LeftTrigger() {
        if (gamepad1.left_trigger > GAMEPAD_TRIGGER_THRESHOLD) {
            return true;
        } else {
            return false;
        }
    }
}



