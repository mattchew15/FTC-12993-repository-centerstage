package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Max;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.accessory.Toggle;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpDown;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDown;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import android.graphics.Path;
import android.provider.Settings;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "SimplicityDrive")
public class SimplicityDrive extends LinearOpMode {

    //Subsystems
    VoltageSensor voltageSensor;
    DriveBase driveBase = new DriveBase();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    ElapsedTime GlobalTimer;
    double sequenceTimer;
    int liftTarget;
    int pitchTarget;
    int intakeTarget;
    int armTarget;

    double backdropRelativeHeight;
    double pureHeight = backdropRelativeHeight * Math.cos(Math.toRadians(30)); // returns pure height tehe
    double outtakeExtensionInches;

    boolean pitching;
    boolean usedTriggers;
    boolean droppedRight;
    boolean droppedLeft;
    boolean climbAdjustIntakeSlides;
    boolean straightenTurret;

    double headingPosition;
    boolean topIsRight;
    boolean topIsLeft;
  //boolean lockLowPosition;

    //Accessories
    LoopTime loopTime = new LoopTime();
    ToggleUpDown miniTurretPositionToggle = new ToggleUpDown(3, 0); // one starts at the centre
    Toggle pivotFlipToggle = new Toggle();
    ToggleUpOrDown fineAdjustSlides = new ToggleUpOrDown(65,65, liftTarget);

    enum OuttakeState { // FSM
        READY,
        INTAKE_EXTENDO,
        INTAKE,
        AFTER_INTAKE,
        INTAKE_TO_TRANSFER,
        TRANSFER_START,
        TRANSFER_END,
        OUTTAKE_ADJUST,
        DEPOSIT,
        RETURN,
        MANUAL_ENCODER_RESET,

        PITCH_RESET,
        CLIMB_START,
        CLIMB_END,
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

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
               module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
             } //


        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        driveBase.initDrivebase(hardwareMap);
        outtakeState = OuttakeState.READY;

        List<Integer> lastTrackingEncPositions = new ArrayList<>(); //
        List<Integer> lastTrackingEncVels = new ArrayList<>(); // idk what this is for but this is what sample mecanum drive does to set localizer

        StandardTrackingWheelLocalizer location = new StandardTrackingWheelLocalizer(hardwareMap,lastTrackingEncPositions,lastTrackingEncVels);

        waitForStart();
        if (opModeIsActive()) {

            GlobalTimer = new ElapsedTime(System.nanoTime());
            GlobalTimer.reset();
            sequenceTimer = GlobalTimer.milliseconds(); // sets up timer for caching

            intakeSubsystem.intakeHardwareSetup();
            outtakeSubsystem.hardwareSetup();
            outtakeSubsystem.encodersReset(); // so that we don't have to do a manual rest if auto messes up1
            intakeSubsystem.intakeSlideMotorEncodersReset();

            driveBase.drivebaseSetup();
            pitching = true;
          //  lockLowPosition = false;
            miniTurretPositionToggle.CycleState = 2; //straight position
            armTarget = 3;

            while (opModeIsActive()) {

                outtakeSubsystem.outtakeReads(false); //we dont use distance sensor in tele
                intakeSubsystem.intakeReads(outtakeState == OuttakeState.INTAKE || outtakeState == OuttakeState.AFTER_INTAKE || outtakeState == OuttakeState.INTAKE_EXTENDO);
                // can be condensed into the one class? - try ita
                loopTime.updateLoopTime(telemetry); // this may or may not work
                driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                driveBase.PowerToggle(gamepad1.left_stick_button);
                pivotFlipToggle.ToggleMode(gamepad2.dpad_up); // this could go on states but it may as well be here cos it is run all the time

                outtakeSequence();

                location.update();
              //  telemetry.addData("RobotVoltage", voltageSensor.getVoltage());
                telemetry.addData("STATE", outtakeState);
                telemetry.addData("IntakeSlidePosition", intakeSubsystem.intakeSlidePosition);
                telemetry.addData("LiftPosition", outtakeSubsystem.liftPosition);
                telemetry.addData("PitchPosition", outtakeSubsystem.pitchPosition);
                telemetry.addData("PitchPositionDegrees", ticksToDegreePitchMotor(outtakeSubsystem.pitchPosition));
                telemetry.addLine("");
                telemetry.addData("IntakeSlideTargetReached", intakeSubsystem.intakeSlideTargetReached());
                telemetry.addData("Colour Sensor front", intakeSubsystem.frontColourSensorValue);
                telemetry.addData("Colour Sensor back", intakeSubsystem.backColourSensorValue);
                telemetry.addLine("");
                telemetry.addData("IntakeMotor Current", intakeSubsystem.intakeCurrent);
                telemetry.addData("IntakeSlideMotor Current", intakeSubsystem.IntakeSlideMotor.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("LiftMotor Current", outtakeSubsystem.LiftMotor.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("PitchMotor Current", outtakeSubsystem.PitchMotor.getCurrent(CurrentUnit.AMPS));
                telemetry.addLine("");
                telemetry.addData("IntakeChuteArmPosition", intakeSubsystem.intakeChuteArmPosition);
                telemetry.addLine("");
                telemetry.addData("MiniTurretPosition", miniTurretPositionToggle.CycleState);
                telemetry.addLine("");
                telemetry.addData("Pitching?", pitching);
                telemetry.addLine("");
                headingPosition = Math.toDegrees(outtakeSubsystem.angleWrap(location.getPoseEstimate().getHeading())); // for some reason previously the angle wrap was in this class
                telemetry.addData("heading", headingPosition); // we use this for the miniturret
                telemetry.update();
                //clears the cache at the end of the loop
                // PhotonCore.CONTROL_HUB.clearBulkCache();
                if (gamepad1.dpad_left){
                    location.setPoseEstimate(new Pose2d(0,0,0));
                }
                if (gamepad2.dpad_down){
                    gamepad1.rumbleBlips(1);
                }
            }
        }
    }

    public void outtakeSequence(){
        switch (outtakeState){
            case READY:

                readyOuttake();
                liftTarget = 0; // this isn't the lift target in this state but rather for when we are fine adjusting lift
                intakeTarget = 0;
                pitchTarget = SIXTY_DEGREE_TICKS;

                intakeClipHoldorNotHold(-3);
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);

                if (GlobalTimer.milliseconds() - sequenceTimer > 100){
                    intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.OPEN);
                    if (gamepad1.right_bumper) {
                        outtakeState = OuttakeState.INTAKE;
                    } else if (gamepad1.left_trigger > 0.2){
                       if (gamepad1.left_bumper) {
                            outtakeState = OuttakeState.INTAKE_EXTENDO;
                            intakeTarget = INTAKE_SLIDE_EXTENDO_TELEOP_CLOSE;
                            sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                        }
                    } else if (gamepad1.left_bumper && gamepad1.left_trigger < 0.2){
                        outtakeState = OuttakeState.INTAKE_EXTENDO;
                        intakeTarget = INTAKE_SLIDE_EXTENDO_TELEOP_FAR;
                        sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                    }

                }

                setIntakeArmHeight();

                if (gamepad1.b||gamepad2.b){
                    intakeSubsystem.intakeSpin(-0.5);
                } else {
                    intakeSubsystem.intakeSpin(0);
                }

                break;

            case INTAKE:
                intakeClipHoldorNotHold(-5);
                intakeSubsystem.intakeSpin(1);
                if (false){ // 10 amps is assumed stalling - should tune
                    intakeSubsystem.intakeSpinState = IntakeSubsystem.IntakeSpinState.SPIT_OUT; // should initiate a reversing sequence
                }
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.OPEN);
                setIntakeArmHeight();

                if (intakeSubsystem.backColourSensorValue > 2500) { // need to tune colour sensor thresholds
                    outtakeState = OuttakeState.AFTER_INTAKE; //reverses intake
                    sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                }
                if (gamepad1.right_trigger > 0.2){
                    sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                    outtakeState = OuttakeState.INTAKE_TO_TRANSFER;
                }
                break;

            case INTAKE_EXTENDO:
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.OPEN);
                setIntakeArmHeight();
                if (GlobalTimer.milliseconds() - sequenceTimer > 90) { // power draw reasons
                    intakeSubsystem.intakeSlideTo(intakeTarget, intakeSubsystem.intakeSlidePosition, 1);
                    intakeSubsystem.intakeSpin(1);
                    if (GlobalTimer.milliseconds() - sequenceTimer > 200){
                        if (false){ // 10 amps is assumed stalling - should tune
                            intakeSubsystem.intakeSpinState = IntakeSubsystem.IntakeSpinState.SPIT_OUT; // should initiate a reversing sequence
                        }
                        if (intakeSubsystem.backColourSensorValue > 2500) { // or a sensor detects this
                            outtakeState = OuttakeState.AFTER_INTAKE;
                            sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                        }
                        if (gamepad1.right_trigger > 0.2){
                            sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                            outtakeState = OuttakeState.INTAKE_TO_TRANSFER;
                        }
                    }
                }
                break;

            case AFTER_INTAKE:

                intakeSubsystem.intakeSpin(1);
                setIntakeArmHeight();
                if (GlobalTimer.milliseconds() - sequenceTimer > 140){
                    if (intakeSubsystem.pixelsInIntake()){ // has to be both because if the second one slides downthe front one will read
                        sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                        outtakeState = OuttakeState.INTAKE_TO_TRANSFER;
                    }
                    if (gamepad1.right_trigger > 0.2){
                        sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                        outtakeState = OuttakeState.INTAKE_TO_TRANSFER;
                    }
                }
                break;

            case INTAKE_TO_TRANSFER:
                liftPositionChange();
                if (GlobalTimer.milliseconds() - sequenceTimer > 20){ // gives time for pixels to orientate
                    intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.HOLDING);
                    if (GlobalTimer.milliseconds() - sequenceTimer > 180) { // time for intake holder to close
                        if (GlobalTimer.milliseconds() - sequenceTimer > 350){
                            intakeSubsystem.intakeSpin(0.35); // helps chute arm go up
                            intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP); // if the intake slides are retracting
                            if (intakeSubsystem.intakeSlidePosition < 5 && GlobalTimer.milliseconds() - sequenceTimer > 400){ // could tune this more finely
                                outtakeState = OuttakeState.TRANSFER_START;
                                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                                sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                            }
                        }else {
                            intakeSubsystem.intakeSpin(-0.6); // reverse intake
                        }
                    }
                }
                intakeClipHoldorNotHold(-8); // backs up intake slides into robot

                break;

            case TRANSFER_START:
                liftPositionChange();

                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.TRANSFER);
                if ((GlobalTimer.milliseconds() - sequenceTimer > 600) || intakeSubsystem.intakeSlidePosition < 2 && intakeSubsystem.chuteDetectorLimitSwitchValue) { //((GlobalTimer.milliseconds() - sequenceTimer > 1500) && (intakeSubsystem.intakeChuteArmPosition < 140)) || GlobalTimer.milliseconds() > 2000
                    intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.OPEN);
                    outtakeState = OuttakeState.TRANSFER_END;
                    sequenceTimer = GlobalTimer.milliseconds(); // resets timer
                }

                if (GlobalTimer.milliseconds() - sequenceTimer > 300){
                    intakeSubsystem.intakeSpin(0);
                } else {
                  intakeSubsystem.intakeSpin(0.3); // helps chute go up
                 }
                intakeClipHoldorNotHold(-8); // backs up intake slides into robot
                break;

            case TRANSFER_END:
                liftPositionChange();
                if (GlobalTimer.milliseconds() - sequenceTimer > 60){ // delay for the transfer to push in
                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                    if (GlobalTimer.milliseconds() - sequenceTimer > 160){ // enough time for grippers to close
                        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                        if (GlobalTimer.milliseconds() - sequenceTimer > 200){
                            intakeSubsystem.intakeSpin(0);
                            if (liftTarget != 0 || pitchTarget!=0) { // basically if we have selected a scoring position
                                outtakeState = OuttakeState.OUTTAKE_ADJUST;
                            }
                        }
                    }
                }
                intakeClipHoldorNotHold(-8); // backs up intake slides into robot

                break;

            case OUTTAKE_ADJUST:
                fineAdjustSlides.upToggle(gamepad2.right_bumper);
                fineAdjustSlides.downToggle(gamepad2.left_bumper);
                liftPositionChange();

                // if adjusting to low have the pitching preset go further in

                if (GlobalTimer.milliseconds() - sequenceTimer > 20){ // makes transfer better?
                    armAdjust(); // depends on the pitch the arm will go to position
                }

               outtakeSubsystem.liftTo(fineAdjustSlides.OffsetTargetPosition, outtakeSubsystem.liftPosition,1);


                if (outtakeSubsystem.liftPosition > LIFT_HITS_WHILE_PITCHING_THRESHOLD) { // so shit doesn't hit the thing when pitching
                    outtakeSubsystem.pitchToInternalPID(pitchTarget,1);
                } else{
                    outtakeSubsystem.pitchToInternalPID(200,1);
                }


              //  miniTurretPositionToggle.cycleToggleUpDown(gamepad1LeftTrigger(),gamepad1RightTrigger());

                // deposit logic
                if (gamepad1.left_trigger > 0.2 || usedTriggers) {
                    usedTriggers = true;
                    if (gamepad1.right_bumper){
                       if (topIsRight){
                           outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.TOP_OPEN);
                       } else if (topIsLeft){
                           outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.BOTTOM_OPEN);
                       }
                       droppedRight = true;
                    }
                    if (gamepad1.left_bumper) {
                        droppedLeft = true;
                        if (topIsLeft){
                            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.TOP_OPEN);
                        } else if (topIsRight){
                            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.BOTTOM_OPEN);
                        }
                    }
                    if (droppedLeft && droppedRight){
                        outtakeState = OuttakeState.DEPOSIT;
                        sequenceTimer = GlobalTimer.milliseconds();// reset timer
                    }
                } else if (gamepad1.right_bumper && !usedTriggers) {
                    sequenceTimer = GlobalTimer.milliseconds();// reset timer
                    outtakeState = OuttakeState.DEPOSIT;
                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                }

                if (gamepad1.dpad_right){
                    straightenTurret = true;
                }

                if (GlobalTimer.milliseconds() - sequenceTimer > 320){ // ensure the arm is all the way out
                    if (!straightenTurret){
                        telemetry.addLine("pointing to backdrop");
                        outtakeSubsystem.miniTurretPointToBackdrop(headingPosition);
                        pivotAdjust();
                    } else {
                        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                    }

                }
                break;

            case DEPOSIT:  // idk what this case does
                // if dropping with the slides further pitched than wait for longer lmao
                if (pitching? GlobalTimer.milliseconds() - sequenceTimer > 200 :GlobalTimer.milliseconds() - sequenceTimer > 400){ // test if manual reset is ok
                    outtakeState = OuttakeState.RETURN;
                    sequenceTimer = GlobalTimer.milliseconds(); // resets time
                }
                break;

            case RETURN:
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                outtakeSubsystem.pitchToInternalPID(SIXTY_DEGREE_TICKS,1);
                intakeSubsystem.intakeSlideInternalPID(-10,1);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.HOLDING);
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                intakeSubsystem.intakeSpin(-0.3);

                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
                if (GlobalTimer.milliseconds() - sequenceTimer > 130){
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                    if (outtakeSubsystem.liftPosition < 80){ // lets slides ramming trick take over in ready state
                        outtakeState = OuttakeState.READY; //:)
                        sequenceTimer = GlobalTimer.milliseconds();


                        //having these here means dont repeatedly set things so its more optimized
                        intakeSubsystem.intakeSpinState = IntakeSubsystem.IntakeSpinState.INTAKE;
                        usedTriggers = false;
                        droppedRight = false;
                        droppedLeft = false;
                        straightenTurret = false;
                    }
                }

                if (!outtakeSubsystem.pitchTargetReached()){ // can still include this even though its not a hardware limitation on new robot
                    outtakeSubsystem.liftToInternalPID(LIFT_HITS_WHILE_PITCHING_THRESHOLD, 1);
                } else {
                   outtakeSubsystem.liftToInternalPID(0,1);
                }


                break;

            case MANUAL_ENCODER_RESET:  // idk what this case does
                outtakeSubsystem.liftMotorRawControl(gamepad2.right_stick_y);
                outtakeSubsystem.pitchMotorRawControl(gamepad2.left_stick_y);
                intakeSubsystem.intakeSlideMotorRawControl(gamepad2.left_trigger-gamepad2.right_trigger);

                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT); // need to create another method to point to backdrop so i don't need to pass through zero each time
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.HOLDING);

                if (gamepad1.left_bumper || gamepad2.left_bumper){
                    intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.TRANSFER);
                } else {
                    intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                }

                if (gamepad2.right_bumper){
                    outtakeSubsystem.liftEncoderReset(); // pitch encoder is already reset
                    intakeSubsystem.intakeSlideMotorEncodersReset();
                    sequenceTimer = GlobalTimer.milliseconds();
                }
                break;


            case CLIMB_START:  // idk what this case does
                if (GlobalTimer.milliseconds() - sequenceTimer > 50){
                    driveBase.climbState(DriveBase.ClimbState.RELEASE);
                }
                if (GlobalTimer.milliseconds() - sequenceTimer > 800){
                    outtakeSubsystem.pitchToInternalPID(UPRIGHT_PITCH_TICKS, 1);
                    intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                } else {
                    outtakeSubsystem.pitchToInternalPID(PITCH_LOW_DEGREE_TICKS, 1);
                }

                if (gamepad1.right_bumper){
                    outtakeState = OuttakeState.CLIMB_END;
                    sequenceTimer = GlobalTimer.milliseconds();
                    intakeSubsystem.intakeSlideInternalPID(250,1); // sets the slides target so following logic works
                }
                break;
            case CLIMB_END:  // idk what this case does
                outtakeSubsystem.pitchToInternalPID(2000, 1);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                if (!climbAdjustIntakeSlides){
                    intakeSubsystem.intakeSlideInternalPID(300,1);
                    if (GlobalTimer.milliseconds() - sequenceTimer > 1000){
                        climbAdjustIntakeSlides = true;
                    }
                } else {
                    intakeSubsystem.intakeSlideMotorRawControl(gamepad1.left_trigger - gamepad1.right_trigger); // balances the robot out
                }
                break;
        }

        // this happens constantly throughout the loop
        if ((gamepad2.b || gamepad1.b) && (outtakeState != OuttakeState.READY)){
            intakeSubsystem.intakeSpin(-0.5);
            outtakeState = OuttakeState.RETURN; // if b is pressed at any state then return to ready
            pivotFlipToggle.ToggleMode = false; // returning should put thing like this to default
            miniTurretPositionToggle.CycleState = 1; // reset turret to straight
        }
        if (gamepad2.share){ // shouldn't need a toggle function for this
            outtakeState = OuttakeState.MANUAL_ENCODER_RESET;
        }

        if (gamepad1.left_trigger > 0.2){ // && outtakeState == OuttakeState.READY
            if (gamepad1.dpad_up){
                driveBase.droneState(DriveBase.DroneState.RELEASE);
            }
            if (gamepad1.dpad_down && outtakeState == OuttakeState.READY){
                outtakeState = OuttakeState.CLIMB_START;
                sequenceTimer = GlobalTimer.milliseconds();
                //intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                driveBase.droneState(DriveBase.DroneState.RELEASE);
                climbAdjustIntakeSlides = false;
            }
        }
        if (gamepad1.dpad_down){
            outtakeSubsystem.pitchToInternalPID(SIXTY_DEGREE_TICKS,1);
        }
    }

    public void setIntakeArmHeight(){ // Change this shit that lotus messed up
        if (gamepad2.a){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
        } else if (gamepad2.x){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE);
        } else if (gamepad2.y){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP);
        }
    }

    public void intakeClipHoldorNotHold(int slideToPosition){
        if (intakeSubsystem.intakeSlidePosition < 2) {
            intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING); // turn the intake slide pid running to pos off to save battery draw
            intakeSubsystem.intakeSlideMotorRawControl(0);
        } else {
            intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // this might break something when as the intake slides won't go in, but stops jittering
            intakeSubsystem.intakeSlideInternalPID(slideToPosition,1);
        }
    }

    public void readyOuttake(){
        outtakeSubsystem.pitchToInternalPID(SIXTY_DEGREE_TICKS,1); // internalPID uses less power draw - integral term is better
        if (gamepad2.left_trigger > 0.2 && gamepad2.left_bumper) { // must hold these 2 things and the slides will slam up and down - dodge fix
            outtakeSubsystem.resetOuttake();
            outtakeSubsystem.outtakeResetState = OuttakeSubsystem.OuttakeResetState.UP; // starts the thing
        } else {
            if (outtakeSubsystem.liftPosition < 4){
                outtakeSubsystem.liftToInternalPID(0,1);
                telemetry.addLine("Normal lift position");
            } else {
                outtakeSubsystem.liftToInternalPID(-200,1);
            }
        }
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT); // need to create another method to point to backdrop so i don't need to pass through zero each time
        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
        outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
        outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
    }
    public void liftPositionChange(){ // if gamepad inputs don't work in this class then pass them through as parameters in the function
        if (gamepad2.y || gamepad1.y){
            liftTarget = LIFT_HIGH_POSITION_TICKS;
            pitchTarget = SIXTY_DEGREE_TICKS;
            armTarget = 3; // this can be optimized definitely
            fineAdjustSlides.clearOffset(liftTarget); // delete this if you want the offset position to be remembered
        } else if (gamepad2.x || gamepad1.x){
            if (pitching){
                liftTarget = LIFT_MEDIUM_POSITION_PITCHING_TICKS;
                armTarget = 2;
                pitchTarget = PITCH_MID_DEGREE_TICKS;
                fineAdjustSlides.clearOffset(liftTarget);
            } else if (!pitching){
                pitchTarget = SIXTY_DEGREE_TICKS;
                armTarget = 3;
                liftTarget = LIFT_MEDIUM_POSITION_TICKS;
                fineAdjustSlides.clearOffset(liftTarget);

            }
        } else if (((gamepad2.a && gamepad2.dpad_down) || gamepad1.a) && pitchTarget != PITCH_MID_DEGREE_TICKS){ // so you can't knock stuff of the backdrop
            if (pitching){
                liftTarget = LIFT_LOW_POSITION_PITCHING_TICKS;
                armTarget = 1;
                pitchTarget = PITCH_LOW_DEGREE_TICKS;
                fineAdjustSlides.clearOffset(liftTarget);
            } else if (!pitching){
                liftTarget = LIFT_LOW_POSITION_TICKS;
                pitchTarget = SIXTY_DEGREE_TICKS;
                armTarget = 3;
                fineAdjustSlides.clearOffset(liftTarget);
            }
        }
    }

    public void armAdjust(){
        if (armTarget == 1){
            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_DOWN);
        } else if (armTarget == 2){
            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_HALF_DOWN);
        } else if (armTarget == 3){
            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_UP);
        }
    }

    public void pivotAdjust(){
        if (gamepad2.left_trigger > 0.2) {
            if (pivotFlipToggle.ToggleMode){
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_LEFT_FLIPPED);
                topIsRight = true;
                topIsLeft = false;
            } else {
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_LEFT);
                topIsLeft = true;
                topIsRight = false;
            }
        } else if (gamepad2.right_trigger > 0.2){
            if (pivotFlipToggle.ToggleMode){
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_RIGHT_FLIPPED);
                topIsLeft = true;
                topIsRight = false;
            } else {
                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_RIGHT);
                topIsRight = true;
                topIsLeft = false;
            }
        } else if (gamepad2.dpad_left){
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
            topIsLeft = true;
            topIsRight = false;
        } else if (gamepad2.dpad_right){
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
            topIsRight = true;
            topIsLeft = false;
        }
    }

    void miniTurretAdjust(){ // have a button on dpad to straighten turret at any time
        if (miniTurretPositionToggle.CycleState == 1){
            outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.DIAGONAL_LEFT);
        } else if (miniTurretPositionToggle.CycleState == 2) {
            outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
        } else if (miniTurretPositionToggle.CycleState == 3) {
            outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.DIAGONAL_RIGHT);
        }
    }

    public void extendOuttake(boolean MaxExtension){
        if (MaxExtension){

            // have an x distance based off the target height
            // calculate a pitch target and slide target based on this using a height variable
            pitchTarget = (int)Math.asin(pureHeight/LIFT_TICKS_FOR_MAX_EXTENSION);
            liftTarget = LIFT_TICKS_FOR_MAX_EXTENSION;
            // height variable is straight up and down - will have a function to convert from height at 30 degrees to actual height
        } else {

            outtakeExtensionInches = 12; // change if we want to extend more when not at max
            pitchTarget = (int)Math.atan2(pureHeight,outtakeExtensionInches);
            liftTarget = (int)Math.sqrt(Math.pow(pureHeight,2)+Math.pow(outtakeExtensionInches,2));
            // have a set x distance
            // calculate a pitch target and slide target based on this using a height variable

           }
    }

    boolean gamepad1RightTrigger(){

        /*
        if (gamepad1.right_trigger > GAMEPAD_TRIGGER_THRESHOLD){
            return true;
        } else {
            return false;
        }
        */
        // old version above if any problems happen, if works delete it
        return (gamepad1.right_trigger > GAMEPAD_TRIGGER_THRESHOLD);
    }
    boolean gamepad1LeftTrigger() {
        /*
        if (gamepad1.left_trigger > GAMEPAD_TRIGGER_THRESHOLD) {
            return true;
        } else {
            return false;
        }
        */
        // old version above if any problems happen, if works delete it

        return (gamepad1.left_trigger > GAMEPAD_TRIGGER_THRESHOLD);
    }
    boolean gamepad2RightTrigger(){
        return (gamepad2.right_trigger > GAMEPAD_TRIGGER_THRESHOLD);
    }
    boolean gamepad2LeftTrigger() {
        return (gamepad1.left_trigger > GAMEPAD_TRIGGER_THRESHOLD);
    }


}



