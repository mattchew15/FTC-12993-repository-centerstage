package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.stat.descriptive.rank.Max;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.accessory.OuttakeInverseKinematics;
import org.firstinspires.ftc.teamcode.system.accessory.Toggle;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleRisingEdge;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpDown;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDown;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

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
    OuttakeInverseKinematics outtakeInverseKinematics = new OuttakeInverseKinematics();

    ElapsedTime GlobalTimer;
    double sequenceTimer;
    int liftTarget;
    int pitchTarget;
    int intakeTarget;
    int armTarget;
    int majorAdjustType;

    double backdropRelativeHeight;
    double pureHeight; // returns pure height tehe
    double outtakeExtensionInches;
    double heightAdjustTimer;
    double adjustHeightCache;

    boolean usedTriggers;
    boolean droppedRight;
    boolean droppedLeft;
    boolean climbAdjustIntakeSlides;
    boolean straightenTurret;
    boolean MaxExtension;
    boolean initialHeightStored;

    double headingPosition;
    boolean topIsRight;
    boolean topIsLeft;
  //boolean lockLowPosition;

    //Accessories
    LoopTime loopTime = new LoopTime();
   // ToggleUpDown miniTurretPositionToggle = new ToggleUpDown(3, 0); // one starts at the centre
    Toggle pivotFlipToggle = new Toggle();
    ToggleUpOrDown fineAdjustHeight = new ToggleUpOrDown(FINE_ADJUST_HEIGHT_INTERVAL, FINE_ADJUST_HEIGHT_INTERVAL, liftTarget);
    ToggleUpOrDown intakeOutBtnLogic = new ToggleUpOrDown(1,1,0);
    ToggleRisingEdge gamepadRightBumperRisingEdge = new ToggleRisingEdge();
    ToggleRisingEdge gamepadLeftBumperRisingEdge = new ToggleRisingEdge();

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
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
            driveBase.drivebaseSetup();
            MaxExtension = true;
            initialHeightStored = false;
            //lockLowPosition = false;
            //miniTurretPositionToggle.CycleState = 2; //straight position
            armTarget = 3;
            backdropRelativeHeight = 15; // change this variable for height

            while (opModeIsActive()) {

                /*
                TODO assign OuttakeInverseKinematics and AprilTagLocalisation 'robotAngle'
                TODO assign AprilTagLocalisation 'aprilTagX' and 'aprilTagY'
                 */

                outtakeSubsystem.outtakeReads(false); //we dont use distance sensor in tele
                intakeSubsystem.intakeReads(outtakeState == OuttakeState.INTAKE || outtakeState == OuttakeState.AFTER_INTAKE || outtakeState == OuttakeState.INTAKE_EXTENDO);
                // can be condensed into the one class? - try ita
                loopTime.updateLoopTime(telemetry); // this may or may not work
                driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                driveBase.PowerToggle(gamepad1.left_stick_button);
                pivotFlipToggle.ToggleMode(gamepad2.dpad_up); // this could go on states but it may as well be here cos it is run all the time
                pureHeight = backdropRelativeHeight * Math.cos(Math.toRadians(30));
                outtakeSequence();

                location.update();
              //  telemetry.addData("RobotVoltage", voltageSensor.getVoltage());
                telemetry.addData("STATE", outtakeState);
                telemetry.addLine("");
                telemetry.addData("liftTarget", liftTarget);
                telemetry.addData("pitchTarget", pitchTarget);
                telemetry.addData("Rail Servo Position", RAIL_SERVO_POSITION);
                telemetry.addLine("");
                telemetry.addData("IntakeSlidePosition", intakeSubsystem.intakeSlidePosition);
                telemetry.addData("LiftPosition", outtakeSubsystem.liftPosition);
                telemetry.addData("PitchPosition", outtakeSubsystem.pitchPosition);
              //  telemetry.addData("PitchPositionDegrees", ticksToDegreePitchMotor(outtakeSubsystem.pitchPosition));
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
                telemetry.addLine("");
                telemetry.addData("intake BTN Logic", intakeOutBtnLogic.OffsetTargetPosition);
                telemetry.addData("backdropRelativeHeight", backdropRelativeHeight);
                telemetry.addData("pureHeight",pureHeight);
                telemetry.addData("outtakeExtensionInches", outtakeExtensionInches);
                //outtakeInverseKinematics.distance = outtakeExtensionInches;

                /*
                if (majorAdjustType == 1){
                    telemetry.addLine("We are adjusting down");
                } else if (majorAdjustType == 2) {
                    telemetry.addLine("Outside of slide constraints");
                } else if (majorAdjustType == 3){
                    telemetry.addLine("Adjusting up");
                }
                 */
                telemetry.addData("majorAdjustType", majorAdjustType);
                telemetry.addLine("");
                //TODO change the heading to the heading of the bot
                //outtakeInverseKinematics.robotAngle = headingPosition;
                headingPosition = 0;//Math.toDegrees(outtakeSubsystem.angleWrap(location.getPoseEstimate().getHeading())); // for some reason previously the angle wrap was in this class
                //telemetry.addData("heading", headingPosition); // we use this for the miniturret
                telemetry.update();
                //clears the cache at the end of the loop
                // PhotonCore.CONTROL_HUB.clearBulkCache();
                if (gamepad1.dpad_left){
                    location.setPoseEstimate(new Pose2d(0,0,0));
                }
                if (gamepad2.dpad_down){
                    gamepad1.rumbleBlips(1);
                }
                outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition,telemetry);
            }
        }
    }

    public void outtakeSequence(){
        switch (outtakeState){
            case READY:

                readyOuttake();
                liftTarget = 0; // this shouldn't affect anything
                intakeTarget = 0;
                pitchTarget = PITCH_DEFAULT_DEGREE_TICKS;

                intakeClipHoldorNotHold(-3);
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);

                if (GlobalTimer.milliseconds() - sequenceTimer > 100){
                    intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
                    if (gamepad1.right_bumper) {
                        outtakeState = OuttakeState.INTAKE;
                    } else if (gamepad1.left_bumper){
                       outtakeState = OuttakeState.INTAKE_EXTENDO;
                       intakeTarget = INTAKE_SLIDE_EXTENDO_TELEOP_CLOSE;
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
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
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
                intakeOutBtnLogic.upToggle(gamepad1.left_bumper);

                if (intakeOutBtnLogic.OffsetTargetPosition == 1){
                    intakeTarget = INTAKE_SLIDE_EXTENDO_TELEOP_CLOSE;
                } else if (intakeOutBtnLogic.OffsetTargetPosition == 2){
                    intakeTarget = INTAKE_SLIDE_EXTENDO_TELEOP_FAR;
                }

                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
                setIntakeArmHeight();
                if (GlobalTimer.milliseconds() - sequenceTimer > 90) { // power draw reasons
                    intakeSubsystem.intakeSlideTo(intakeTarget, intakeSubsystem.intakeSlidePosition, 1);
                    intakeSubsystem.intakeSpin(1);
                    if (GlobalTimer.milliseconds() - sequenceTimer > 200){
                        if (false){ // 10 amps is assumed stalling - should tune
                            intakeSubsystem.intakeSpinState = IntakeSubsystem.IntakeSpinState.SPIT_OUT; // should initiate a reversing sequence
                        }
                        if (intakeSubsystem.backColourSensorValue > 2000) { // or a sensor detects this
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
                liftPositionChange(false,false);
                if (GlobalTimer.milliseconds() - sequenceTimer > 20){ // gives time for pixels to orientate
                    intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
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
                liftPositionChange(false, false); // we dont want d1 control in this state

                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.TRANSFER);
                if ((GlobalTimer.milliseconds() - sequenceTimer > 700) || intakeSubsystem.intakeSlidePosition < 2 && intakeSubsystem.chuteDetectorLimitSwitchValue) { //((GlobalTimer.milliseconds() - sequenceTimer > 1500) && (intakeSubsystem.intakeChuteArmPosition < 140)) || GlobalTimer.milliseconds() > 2000
                    intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);

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
                liftPositionChange(false, false);

                if (GlobalTimer.milliseconds() - sequenceTimer > 60){ // delay for the transfer to push in
                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                    if (GlobalTimer.milliseconds() - sequenceTimer > 160){ // enough time for grippers to close
                        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                        if (GlobalTimer.milliseconds() - sequenceTimer > 200){
                            intakeSubsystem.intakeSpin(0);
                            if (gamepadRightBumperRisingEdge.mode(gamepad1.right_bumper)) { //.mode is returns a boolean
                                outtakeState = OuttakeState.OUTTAKE_ADJUST;
                                MaxExtension = true;
                                extendOuttake();
                            } else if (gamepadLeftBumperRisingEdge.mode(gamepad1.left_bumper)){
                                outtakeState = OuttakeState.OUTTAKE_ADJUST;
                                MaxExtension = false;
                                extendOuttake();
                            }
                        }
                    }
                }
                intakeClipHoldorNotHold(-8); // backs up intake slides into robot

                break;

            case OUTTAKE_ADJUST:
                liftPositionChange(true,true);
                adjustRail(); // if statements for bumpers
                // driver 2 fine adjusts rail
                outtakeSubsystem.fineAdjustRail(gamepad1.right_trigger-gamepad1.left_trigger, GlobalTimer.milliseconds());

               // fineAdjustHeight.upToggle(gamepad2.right_bumper);
               // fineAdjustHeight.downToggle(gamepad2.left_bumper);
                fineAdjustHeight(gamepad2.right_stick_y); // this shouldn't conflict with bumpers because of rising edge detector logic

                // if adjusting to low have the pitching preset go further in

                if (GlobalTimer.milliseconds() - sequenceTimer > 20){ // makes transfer better?
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE);
                    outtakeSubsystem.setOuttakeRailServo(RAIL_SERVO_POSITION); // RAIL_SERVO_POSITION SHOULD BE REMEMBERED
                }

               // pitchTarget = (int)outtakeInverseKinematics.pitchEnd(backdropRelativeHeight);
               // liftTarget = (int)outtakeInverseKinematics.slideEnd(backdropRelativeHeight);
               // backdropRelativeHeight = fineAdjustHeight.OffsetTargetPosition; // this should work

                //RAIL_SERVO_POSITION = (int)outtakeInverseKinematics.railEnd(something,backdropRelativeHeight,RAIL_SERVO_POSITION);
                // idk if we need specific rail adjustment here
                outtakeSubsystem.liftTo(liftTarget, outtakeSubsystem.liftPosition,1);


                if (outtakeSubsystem.liftPosition > LIFT_HITS_WHILE_PITCHING_THRESHOLD) { // so shit doesn't hit the thing when pitching
                    outtakeSubsystem.pitchTo(pitchTarget,outtakeSubsystem.pitchEncoderPosition,1);
                } else{
                    outtakeSubsystem.pitchTo(PITCH_DEFAULT_DEGREE_TICKS,outtakeSubsystem.pitchEncoderPosition,1);
                }

                // Drop logic
                 if (gamepadRightBumperRisingEdge.mode(gamepad1.right_bumper)) {
                    sequenceTimer = GlobalTimer.milliseconds();// reset timer
                    outtakeState = OuttakeState.DEPOSIT;
                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                } else {
                     if (gamepad1.right_stick_button){
                         if (topIsRight){
                             outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.TOP_OPEN);
                         } else if (topIsLeft){
                             outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.BOTTOM_OPEN);
                         }
                         droppedRight = true;
                     }
                     if (gamepad1.left_stick_button) {
                         droppedLeft = true;
                         if (topIsLeft){
                             outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.TOP_OPEN);
                         } else if (topIsRight){
                             outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.BOTTOM_OPEN);
                         }
                     }
                 }
                if (droppedLeft && droppedRight){
                    outtakeState = OuttakeState.DEPOSIT;
                    sequenceTimer = GlobalTimer.milliseconds();// reset timer
                }
                // drop logic

                if (gamepad1.dpad_right){
                    straightenTurret = true;
                }

                if (GlobalTimer.milliseconds() - sequenceTimer > 300){ // ensure the arm is all the way out
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
                if (outtakeSubsystem.pitchPosition > 50? GlobalTimer.milliseconds() - sequenceTimer > 200 :GlobalTimer.milliseconds() - sequenceTimer > 400){ // test if manual reset is ok
                    outtakeState = OuttakeState.RETURN;
                    sequenceTimer = GlobalTimer.milliseconds(); // resets time
                }
                break;

            case RETURN:
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                outtakeSubsystem.pitchTo(PITCH_DEFAULT_DEGREE_TICKS,outtakeSubsystem.pitchEncoderPosition,1);
                intakeSubsystem.intakeSlideInternalPID(-3,1);
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                intakeSubsystem.intakeSpin(-0.3);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);

                outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);

                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
                if (GlobalTimer.milliseconds() - sequenceTimer > (Math.abs(RAIL_SERVO_POSITION-RAIL_CENTER_POS)*700) + 130){
                    // the line above will retract the arm based on how far off the rail servo is at difference timing -> y = mx + c basically
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                    if (outtakeSubsystem.liftPosition < 3){ // lets slides ramming trick take over in ready state
                        outtakeState = OuttakeState.READY; //:)
                        sequenceTimer = GlobalTimer.milliseconds();

                        //having these here means dont repeatedly set things so its more optimized
                        intakeSubsystem.intakeSpinState = IntakeSubsystem.IntakeSpinState.INTAKE;
                        usedTriggers = false;
                        droppedRight = false;
                        droppedLeft = false;
                        straightenTurret = false;
                        initialHeightStored = false; // resets this each cycle
                        outtakeExtensionInches = 12; // change if we want to extend more when not at max
                        backdropRelativeHeight += FINE_ADJUST_HEIGHT_INTERVAL;
                        intakeOutBtnLogic.OffsetTargetPosition = 0;
                        //fineAdjustSlides.clearOffset(liftTarget); // delete this if you want the offset position to be remembered
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
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);

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


            case CLIMB_START:
                if (GlobalTimer.milliseconds() - sequenceTimer > 800){
                    outtakeSubsystem.pitchTo(PITCH_CLIMB_TICKS, outtakeSubsystem.pitchEncoderPosition,1);
                    intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                } else {
                    outtakeSubsystem.liftTo(LIFT_INCHES_FOR_MAX_EXTENSION,outtakeSubsystem.liftPosition,1);
                    outtakeSubsystem.pitchTo(PITCH_DEFAULT_DEGREE_TICKS,outtakeSubsystem.pitchEncoderPosition,1);
                }

                if (gamepad1.right_bumper){
                    outtakeState = OuttakeState.CLIMB_END;
                    sequenceTimer = GlobalTimer.milliseconds();
                }
                break;
            case CLIMB_END:  // idk what this case does
                outtakeSubsystem.pitchTo(20,outtakeSubsystem.pitchEncoderPosition, 1);
                outtakeSubsystem.liftToInternalPID(5,1); // not quite in
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

        if ((gamepad2.b || gamepad1.b) && (outtakeState != OuttakeState.READY) && (outtakeState != OuttakeState.MANUAL_ENCODER_RESET)){
            // can't reset if in manual reset lmao
            //intakeSubsystem.intakeSpin(-0.3);
            outtakeState = OuttakeState.RETURN; // if b is pressed at any state then return to ready
            pivotFlipToggle.ToggleMode = false; // returning should put thing like this to default
        }
        if (gamepad2.share){ // shouldn't need a toggle function for this
            outtakeState = OuttakeState.MANUAL_ENCODER_RESET;
        }

        if (gamepad1.left_trigger > 0.2){ // && outtakeState == OuttakeState.READY
            if (gamepad1.dpad_up){
                driveBase.droneState(DriveBase.DroneServoState.RELEASE);
            }
            if (gamepad1.dpad_right && outtakeState == OuttakeState.READY){
                outtakeState = OuttakeState.CLIMB_START;
                sequenceTimer = GlobalTimer.milliseconds();
                //intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                driveBase.droneState(DriveBase.DroneServoState.RELEASE);
                climbAdjustIntakeSlides = false;
            }
        }
    }

    public void setIntakeArmHeight(){ // Change this shit that lotus messed up
        if (gamepad2.a || gamepad1.a){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
        } else if (gamepad2.x || gamepad1.x){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE);
        } else if (gamepad2.y || gamepad1.y){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP);
        }
    }

    public void intakeClipHoldorNotHold(int slideToPosition){
        if (intakeSubsystem.intakeSlidePosition < 2) {
            intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING); // turn the intake slide pid running to pos off to save battery draw
            //intakeSubsystem.intakeSlideMotorRawControl(0);
            intakeSubsystem.intakeSlideInternalPID(0,1);
        } else {
            intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // this might break something when as the intake slides won't go in, but stops jittering
            intakeSubsystem.intakeSlideInternalPID(slideToPosition,1);
        }
    }

    public void adjustRail(){
        if (gamepad1.dpad_down){
            outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
        }
        /*
        else if (gamepad1LeftTrigger()){
            outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.LEFT);
        } else if (gamepad1RightTrigger()){
            outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.RIGHT);
        }
         */
    }

    public void readyOuttake(){
        outtakeSubsystem.pitchTo(PITCH_DEFAULT_DEGREE_TICKS,outtakeSubsystem.pitchEncoderPosition,1); // internalPID uses less power draw - integral term is better
        if (gamepad2.left_trigger > 0.2 && gamepad2.left_bumper) { // must hold these 2 things and the slides will slam up and down - dodge fix
            outtakeSubsystem.resetOuttake();
            outtakeSubsystem.outtakeResetState = OuttakeSubsystem.OuttakeResetState.UP; // starts the thing
        } else {
            if (outtakeSubsystem.liftPosition < 0.5){ // 0.5 inches remember that
                outtakeSubsystem.liftToInternalPID(0,1);
                telemetry.addLine("Normal lift position");
            } else {
                outtakeSubsystem.liftToInternalPID(-5,1);
            }
        }
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT); // need to create another method to point to backdrop so i don't need to pass through zero each time
        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
        outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
        outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
    }

    public void extendOuttake(){ // this only really runs once
        if (MaxExtension){
            // have an x distance based off the target height
            // calculate a pitch target and slide target based on this using a height variable
            if (pureHeight < LIFT_INCHES_FOR_MAX_EXTENSION){
                pitchTarget = (int)Math.toDegrees(Math.asin(pureHeight/LIFT_INCHES_FOR_MAX_EXTENSION));
            } else {
                pitchTarget = 60;
            }
            liftTarget = LIFT_INCHES_FOR_MAX_EXTENSION;
            outtakeExtensionInches = liftTarget * Math.cos(Math.toRadians(pitchTarget)); // this will give an estimate for the outtake extension
            // height variable is straight up and down - will have a function to convert from height at 30 degrees to actual height
        } else {
            // maximun outtake extension is fixed
            pitchTarget = (int)Math.atan2(pureHeight,outtakeExtensionInches);
            if(pitchTarget < 20){
                pitchTarget = 20;
            } else if (pitchTarget > 60){
                pitchTarget = 60;
            }
            liftTarget = (int)Math.sqrt(Math.pow(pureHeight,2)+Math.pow(outtakeExtensionInches,2));
            if (liftTarget > 25){
                liftTarget = 25;
            }
            // have a set x distance
            // calculate a pitch target and slide target based on this using a height variable

        }
    }

    // This function isn't run in ready state
    public void liftPositionChange(boolean D1controls, boolean afterExtended){ // if gamepad inputs don't work in this class then pass them through as parameters in the function
        //TODO rising edge detector for this so intialheight is only stored once - otherwise the adjusting logic wont work
        if (gamepad2.y || gamepad1.y && D1controls){
            adjustHeightCache = HIGH_BACKDROP_PRESET_INCHES;
            //initialHeightStored = false;
            adjustingToMajorPresetLogic(afterExtended); // this should work for all cases - very complex and will require alot of telemetry
        } else if (gamepad2.x || gamepad1.x && D1controls){
            adjustHeightCache = MID_BACKDROP_PRESET_INCHES; // this part can be changed in throughout any part of the program
            //initialHeightStored = false;
            adjustingToMajorPresetLogic(afterExtended);
        } else if ((gamepad2.a && gamepad2.dpad_down || gamepad1.a && D1controls)){ // so you can't knock stuff of the backdrop
            adjustHeightCache = LOW_BACKDROP_PRESET_INCHES;
            //initialHeightStored = false;
            adjustingToMajorPresetLogic(afterExtended);
        }
    }
    public void adjustingToMajorPresetLogic(boolean afterExtended){

        if (afterExtended && !initialHeightStored){
            initialHeightStored = true; // this is reset every cycle
            if (backdropRelativeHeight - adjustHeightCache > 0){ // this means that we are adjusting down
                MaxExtension = false; // best way to explain this is that as soon as the height is selected the outtake extension limit will also be calculated
                outtakeExtensionInches = outtakeExtensionInches - ((backdropRelativeHeight - adjustHeightCache)/Math.tan(Math.toRadians(60)));
                // line above ensures that endefector stays in line with backdrop when adjusting
                backdropRelativeHeight = adjustHeightCache;
                extendOuttake(); // only need to run this once to set new targets
                majorAdjustType = 1;
            } else { // we are adjusting up so we can just set targets
                // check if we have reached the physical limits by the slides with current outtakeextensionInches
                backdropRelativeHeight = adjustHeightCache;
                if ((int)(Math.sqrt(Math.pow(pureHeight,2)+Math.pow(outtakeExtensionInches,2))) > LIFT_INCHES_FOR_MAX_EXTENSION){
                    MaxExtension = true;
                    extendOuttake();
                    // extend the outtake to its limits for max height
                    majorAdjustType = 2;

                } else {
                    MaxExtension = false;
                    // extend the outtake using the current outtakeextensioninches value - should just go up 30 degrees
                    // technically you should move this value forwards similar how we do it above so that it follows the backdrop
                    extendOuttake();
                    majorAdjustType = 3;
                }
            }
            // this way we will set max extension limits so that we can't increase our outtake extension if we try and readjust
            // Max extension will be set to true when i select the outtake button already, so this maxextension=false will only matter when we are fine adjusting, which is what we want
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

    public void fineAdjustHeight(double fineAdjust){
        if (GlobalTimer.milliseconds() - heightAdjustTimer > 100){ // only set the position every 15 ms, once achieved cache the timer value
            backdropRelativeHeight += fineAdjust * 0.8;
            heightAdjustTimer = GlobalTimer.milliseconds(); // cache the value of the outtakerailadjust
        }
    }

    boolean gamepad1RightTrigger(){
        return (gamepad1.right_trigger > GAMEPAD_TRIGGER_THRESHOLD);
    }
    boolean gamepad1LeftTrigger() {
        return (gamepad1.left_trigger > GAMEPAD_TRIGGER_THRESHOLD);
    }
    boolean gamepad2RightTrigger(){
        return (gamepad2.right_trigger > GAMEPAD_TRIGGER_THRESHOLD);
    }
    boolean gamepad2LeftTrigger() {
        return (gamepad1.left_trigger > GAMEPAD_TRIGGER_THRESHOLD);
    }


}



