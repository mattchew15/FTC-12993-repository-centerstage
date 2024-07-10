package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.system.accessory.Log;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;

import org.firstinspires.ftc.teamcode.system.accessory.imu.ImuThread;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeInverseKinematics;
import org.firstinspires.ftc.teamcode.system.accessory.Toggle;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleRisingEdge;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDown;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import java.security.KeyStore;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "SimplicityDrive")
public class SimplicityDrive extends LinearOpMode {

    //Subsystems
    VoltageSensor voltageSensor;
    DriveBase driveBase = new DriveBase(telemetry);
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
    int pivotState;
    boolean outtakeReset;

    double intakeClipTimer;
    double backdropRelativeHeight;
    double pureHeight; // returns pure height tehe
    double prevPureHeight;
    double outtakeExtensionInches;
    double heightAdjustTimer;
    double intakeSlideAdjustTimer;
    double intakeSlideAdjustTarget;
    double adjustHeightCache;
    double prevExtensionValue;
    double globalTimer;

    boolean usedTriggers;
    boolean droppedRight;
    boolean droppedLeft;
    boolean climbAdjustIntakeSlides;
    boolean straightenTurret;
    boolean MaxExtension;
    boolean initialHeightStored;
    boolean initialPivot;
    boolean increaseHeight;
    boolean transferEarly;
    boolean reArrangePixels;

    boolean rightBumperPreExtend;
    boolean leftBumperPreExtend;

    double headingPosition;
    boolean topIsRight;
    boolean topIsLeft;
  //boolean lockLowPosition;

    //Accessories
    LoopTime loopTime = new LoopTime();
   // ToggleUpDown miniTurretPositionToggle = new ToggleUpDown(3, 0); // one starts at the centre
    Toggle pivotFlipToggle = new Toggle();
    Toggle changeExtension = new Toggle();
    ToggleUpOrDown fineAdjustHeight = new ToggleUpOrDown(FINE_ADJUST_HEIGHT_INTERVAL, FINE_ADJUST_HEIGHT_INTERVAL, (int)backdropRelativeHeight); // this isn't updating throughout the loop
    ToggleUpOrDown intakeOutBtnLogic = new ToggleUpOrDown(1,1,0);
    ToggleRisingEdge gamepadRightBumperRisingEdge = new ToggleRisingEdge();
    ToggleRisingEdge gamepadLeftBumperRisingEdge = new ToggleRisingEdge();

    ToggleRisingEdge highPreset = new ToggleRisingEdge();
    ToggleRisingEdge midPreset = new ToggleRisingEdge();
    ToggleRisingEdge lowPreset = new ToggleRisingEdge();

    Log log = new Log("SimplicityDrive PowerDraw", true);

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
    ImuThread imuThread;
    IMU imu;

    @Override
    public void runOpMode() {

        /*
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
         */

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
               module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
             } //


        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        driveBase.initDrivebase(hardwareMap);
        outtakeState = OuttakeState.READY;

//        imu = hardwareMap.get(BHI260IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        imuThread = new ImuThread(hardwareMap);
        imuThread.initImuThread();
        imuThread.startThread(this);
        List<Integer> lastTrackingEncPositions = new ArrayList<>(); //
        List<Integer> lastTrackingEncVels = new ArrayList<>(); // idk what this is for but this is what sample mecanum drive does to set localizer
        //StandardTrackingWheelLocalizer location = new StandardTrackingWheelLocalizer(hardwareMap,lastTrackingEncPositions,lastTrackingEncVels);
        //TwoWheelTrackingLocalizer location = new TwoWheelTrackingLocalizer(hardwareMap, sampleMecanumDrive);

        waitForStart();
        if (opModeIsActive()) {

            GlobalTimer = new ElapsedTime(System.nanoTime());
            GlobalTimer.reset();
            sequenceTimer = GlobalTimer.milliseconds(); // sets up timer for caching
            intakeSubsystem.intakeHardwareSetup();
            outtakeSubsystem.hardwareSetup();
            outtakeSubsystem.encodersReset(); // resets just the lift encoder
            intakeSubsystem.intakeSlideMotorEncodersReset();
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
            driveBase.drivebaseSetup();
            MaxExtension = true;
            initialHeightStored = false;
            initialPivot = true;
            transferEarly = false;
            armTarget = 3;
            backdropRelativeHeight = 12; // change this variable for height
            outtakeSubsystem.cacheInitialPitchValue();
            RAIL_SERVO_POSITION = RAIL_CENTER_POS;

            while (opModeIsActive()) {

                for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
                    module.clearBulkCache();
                }

                globalTimer = GlobalTimer.milliseconds();
                pureHeight = backdropRelativeHeight * Math.cos(Math.toRadians(30));
                outtakeSubsystem.outtakeReads(false); //we don't use distance sensor in tele
                intakeSubsystem.intakeReads(outtakeState == OuttakeState.INTAKE || outtakeState == OuttakeState.AFTER_INTAKE || outtakeState == OuttakeState.INTAKE_EXTENDO);
                // can be condensed into the one class? - try it

                loopTime.updateLoopTime(telemetry); // this may or may not work
                driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                pivotFlipToggle.ToggleMode(gamepad2.dpad_up); // this could go on states but it may as well be here cos it is run all the time
                adjustRail();
                pivotAdjust();
                outtakeSequence();
                outtakeInverseKinematics.distance = outtakeExtensionInches;

                telemetry.addData("STATE", outtakeState);
                //telemetry.addData("arm height")
                /*
                telemetry.addLine("");
                telemetry.addData("liftTarget", liftTarget);
                telemetry.addData("pitchTarget", pitchTarget);
                telemetry.addLine("");
                telemetry.addData("IntakeSlidePosition", intakeSubsystem.intakeSlidePosition);

                telemetry.addData("PitchPosition", outtakeSubsystem.pitchEncoderPosition);
              //  telemetry.addData("PitchPositionDegrees", ticksToDegreePitchMotor(outtakeSubsystem.pitchPosition));
                telemetry.addLine("");
              *///  telemetry.addData("IntakeSlideTargetReached", intakeSubsystem.intakeSlideTargetReached());
                //telemetry.addData("Colour Sensor front", intakeSubsystem.frontColourSensorValue);
                //telemetry.addData("Colour Sensor back", intakeSubsystem.backColourSensorValue);
                //telemetry.addData("headingPosition", headingPosition);
               // telemetry.addData("Raw heading", AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));

                //telemetry.addData("IntakeSlideMotor Current", intakeSubsystem.IntakeSlideMotor.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("LiftMotor Current", outtakeSubsystem.LiftMotor.getCurrent(CurrentUnit.AMPS));
                //telemetry.addData("PitchMotor Current", outtakeSubsystem.PitchMotor.getCurrent(CurrentUnit.AMPS));

                if (gamepad1.right_stick_button){
                    intakeSubsystem.intakeSpin(-1);
                } if (gamepad1.dpad_down || (gamepad1.left_stick_button && outtakeState != OuttakeState.OUTTAKE_ADJUST)){
                    RAIL_SERVO_POSITION = RAIL_CENTER_POS;
                }
                /*
                log.addData(
                        outtakeSubsystem.LiftMotor.getCurrent(CurrentUnit.AMPS),
                        outtakeSubsystem.PitchMotor.getCurrent(CurrentUnit.AMPS),
                        intakeSubsystem.IntakeSlideMotor.getCurrent(CurrentUnit.AMPS),
                        intakeSubsystem.IntakeMotor.getCurrent(CurrentUnit.AMPS),
                        driveBase.FL.getCurrent(CurrentUnit.AMPS),
                        driveBase.FR.getCurrent(CurrentUnit.AMPS),
                        driveBase.BL.getCurrent(CurrentUnit.AMPS),
                        driveBase.BR.getCurrent(CurrentUnit.AMPS));

                log.addData(outtakeState);
                 */
                headingPosition = Math.toDegrees(outtakeSubsystem.angleWrap(imuThread.getImuAngle()));//Math.toDegrees(outtakeSubsystem.angleWrap(location.getPoseEstimate().getHeading())); // for some reason previously the angle wrap was in this class
                //log.update();
                //clears the cache at the end of the loop
                // PhotonCore.CONTROL_HUB.clearBulkCache();
                if (gamepad1.dpad_left){
                    imuThread.initImuThread();
                    imuThread.resetYaw();
                    //imu.close();
//                    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
//                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                            RevHubOrientationOnRobot.UsbFacingDirection.UP)));
//                    imu.resetYaw();
                }
                if (gamepad2.dpad_down){
                    gamepad1.rumbleBlips(1);
                }
                if (outtakeState != OuttakeState.MANUAL_ENCODER_RESET){
                    outtakeSubsystem.outtakePitchServoKeepToPitch(outtakeSubsystem.pitchEncoderPosition);
                }
                pureHeight = prevPureHeight;
                telemetry.update();
            }
        }
      //  log.close();
    }

    public void outtakeSequence(){
        switch (outtakeState){
            case READY:

                readyOuttake();
                liftTarget = 0; // this shouldn't affect anything
                intakeTarget = 0;
                pitchTarget = PITCH_DEFAULT_DEGREE_TICKS;

                intakeClipHoldorNotHold(-100, 8);
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);

                if (delay(100)){
                    intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
                    if (gamepad1.right_bumper) {
                        outtakeState = OuttakeState.INTAKE;
                    } else if (gamepad1.left_bumper){
                       outtakeState = OuttakeState.INTAKE_EXTENDO;
                       intakeTarget = INTAKE_SLIDE_EXTENDO_TELEOP_CLOSE;
                       intakeOutBtnLogic.upToggle(gamepad1.left_bumper);
                       resetTimer(); // resets timer
                    }

                }

                setIntakeArmHeight();

                if (gamepad1.b||gamepad2.b){
                    intakeSubsystem.intakeSpin(-0.6);
                } else {
                    intakeSubsystem.intakeSpin(0);
                }

                break;

            case INTAKE:
                if (gamepad1.left_bumper){
                    resetTimer(); // resets timer
                    outtakeState = OuttakeState.INTAKE_EXTENDO;
                    intakeTarget = INTAKE_SLIDE_EXTENDO_TELEOP_CLOSE;
                    intakeOutBtnLogic.upToggle(gamepad1.left_bumper);
                }
                intakeClipHoldorNotHold(-100, 6);
                intakeSubsystem.intakeSpin(1);
                if (false){ // 10 amps is assumed stalling - should tune
                    intakeSubsystem.intakeSpinState = IntakeSubsystem.IntakeSpinState.SPIT_OUT; // should initiate a reversing sequence
                }
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
                setIntakeArmHeight();

                if (intakeSubsystem.backColourSensorValue > 1000) { // need to tune colour sensor thresholds
                    outtakeState = OuttakeState.AFTER_INTAKE; //reverses intake
                    resetTimer(); // resets timer
                    gamepad1.rumbleBlips(1);
                }
                if (gamepad1.right_trigger > 0.2){
                    resetTimer(); // resets timer
                    outtakeState = OuttakeState.INTAKE_TO_TRANSFER;
                }
                break;

            case INTAKE_EXTENDO:
                intakeOutBtnLogic.upToggle(gamepad1.left_bumper);
                intakeOutBtnLogic.downToggle(gamepad1.right_bumper);
                if (intakeOutBtnLogic.OffsetTargetPosition == 1){
                    intakeTarget = INTAKE_SLIDE_EXTENDO_TELEOP_CLOSE;
                } else if (intakeOutBtnLogic.OffsetTargetPosition == 2){
                    intakeTarget = INTAKE_SLIDE_EXTENDO_TELEOP_FAR;
                }

                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
                setIntakeArmHeight();
                if (delay(90)) { // power draw reasons
                    intakeSubsystem.intakeSlideInternalPID(intakeTarget, 1);
                    intakeSubsystem.intakeSpin(1);
                    if (delay(200)){
                        if (false){ // 10 amps is assumed stalling - should tune
                            intakeSubsystem.intakeSpinState = IntakeSubsystem.IntakeSpinState.SPIT_OUT; // should initiate a reversing sequence
                        }
                        if (intakeSubsystem.backColourSensorValue > 1000) { // or a sensor detects this
                            outtakeState = OuttakeState.AFTER_INTAKE;
                            resetTimer(); // resets timer
                            gamepad1.rumbleBlips(1);
                        }
                        if (gamepad1.right_trigger > 0.2){
                            resetTimer(); // resets timer
                            outtakeState = OuttakeState.INTAKE_TO_TRANSFER;
                        }
                    }
                }
                break;


            case AFTER_INTAKE:
                intakeOutBtnLogic.upToggle(gamepad1.left_bumper);
                intakeOutBtnLogic.downToggle(gamepad1.right_bumper);
                if (intakeOutBtnLogic.OffsetTargetPosition == 1){
                    intakeTarget = INTAKE_SLIDE_EXTENDO_TELEOP_CLOSE;
                } else if (intakeOutBtnLogic.OffsetTargetPosition == 2){
                    intakeTarget = INTAKE_SLIDE_EXTENDO_TELEOP_FAR;
                }
                intakeSubsystem.intakeSlideInternalPID(intakeTarget, 1);
                intakeSubsystem.intakeSpin(1);

                setIntakeArmHeight();
                if (delay(40)){
                    if (intakeSubsystem.pixelsInIntake()){ // has to be both because if the second one slides downthe front one will read
                        resetTimer(); // resets timer
                        outtakeState = OuttakeState.INTAKE_TO_TRANSFER;
                        gamepad1.rumbleBlips(1);
                    }
                    if (gamepad1.right_trigger > 0.2){
                        resetTimer(); // resets timer
                        outtakeState = OuttakeState.INTAKE_TO_TRANSFER;
                        gamepad1.rumbleBlips(1);
                    }
                }
                break;

            case INTAKE_TO_TRANSFER:
                liftPositionChange(false,false);
                if (delay(80)){ // gives time for pixels to orientate
                    intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
                    if (delay(170)) { // time for intake holder to close
                        if (delay(380)){
                            //intakeSubsystem.intakeSpin(1); // helps chute arm go up
                            intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP); // if the intake slides are retracting
                            if (delay(380)){ // could tune this more finely
                                outtakeState = OuttakeState.TRANSFER_START;
                                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                                resetTimer(); // resets timer
                            }
                        } else {
                            intakeSubsystem.intakeSpin(-1); // reverse intake
                        }
                    }
                }
                intakeClipHoldorNotHold(-100,8); // backs up intake slides into robot
                break;

            case TRANSFER_START:
                gamepad1.rumbleBlips(1);
                if (gamepad1.right_stick_button){
                    intakeSubsystem.intakeSlideMotorEncodersReset();
                }
                bumperPressed();
                liftPositionChange(false,false); // we dont want d1 control in this state
                liftDown(0.07);
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.TRANSFER);
                telemetry.addData("lift position", ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition));
                if (((delay(460)) && intakeSubsystem.intakeSlidePosition < 12 || (intakeSubsystem.intakeSlidePosition < 29 && intakeOutBtnLogic.OffsetTargetPosition == 2)) && ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) < 0.2) { //((delay() 1500) && (intakeSubsystem.intakeChuteArmPosition < 140)) || globalTimer > 2000
                    outtakeState = OuttakeState.TRANSFER_END;
                    gamepad2.rumbleBlips(1);
                    intakeSubsystem.intakeSpin(0);
                    resetTimer();
                }

                if (delay(0)){
                    intakeSubsystem.intakeSpin(0);
                } else {
                  //intakeSubsystem.intakeSpin(0.8);
                 }
                intakeClipHoldorNotHoldWithoutPowerCutout(-100);
                break;

            case TRANSFER_END:
                bumperPressed();
                liftPositionChange(false, false);
                //intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.OPEN);
                if (delay(45)){ // delay for the transfer to push in
                    outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                    if (delay(170)){ // enough time for grippers to close
                       //intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                        if (delay(160)){ //&& ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) < 0.1
                            //outtakeSubsystem.liftToInternalPID(0,1);
                            if (delay(240)){
                                //outtakeSubsystem.liftMotorRawControl(0);
                                outtakeSubsystem.liftToInternalPID(0,1);
                                intakeSubsystem.intakeSpin(0);
                            }
                            intakeClipHoldorNotHold(-8,7); // backs up intake slides into robot

                            if (gamepadRightBumperRisingEdge.mode(gamepad1.right_bumper) || rightBumperPreExtend) { //.mode is returns a boolean
                                outtakeState = OuttakeState.OUTTAKE_ADJUST;
                                changeExtension.ToggleMode = true;
                                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                                resetTimer();// reset timer
                                MaxExtension = true;
                                extendOuttake();
                            } else if (gamepadLeftBumperRisingEdge.mode(gamepad1.left_bumper) || leftBumperPreExtend){
                                outtakeState = OuttakeState.OUTTAKE_ADJUST;
                                changeExtension.ToggleMode = true;
                                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                                resetTimer();// reset timer
                                MaxExtension = false;
                                outtakeSubsystem.liftToInternalPID(0,1);
                                extendOuttake();
                            }
                        }else {
                            liftDown(0.05);
                            intakeClipHoldorNotHoldWithoutPowerCutout(-8); // backs up intake slides into robot
                        }
                    } else {
                        intakeClipHoldorNotHoldWithoutPowerCutout(-8); // backs up intake slides into robot
                        liftDown(0.05);
                    }
                } else {
                    intakeClipHoldorNotHoldWithoutPowerCutout(-8); // backs up intake slides into robot
                    liftDown(0.05);
                }
                break;

            case OUTTAKE_ADJUST:

                if (changeExtension.ToggleMode){
                    outtakeExtensionInches = MIN_OUTTAKE_EXTENSION_INCHES;
                    //RAIL_SERVO_POSITION = RAIL_CENTER_POS;
                } else {
                    outtakeExtensionInches = prevExtensionValue;
                    prevExtensionValue = outtakeExtensionInches; // this should work idk
                }

                intakeClipHoldorNotHold(-10,8); // otherwise we are setting internalPID is still doing things
                if (delay(500)){
                    intakeSubsystem.intakeSpin(0);
                }else{
                    intakeSubsystem.intakeSpin(-0.8);
                }

                changeExtension.ToggleMode(gamepad1.dpad_up);
                liftPositionChange(true,true);
                // tinkos major adjustment
                // driver 2 fine adjusts rail
                outtakeSubsystem.fineAdjustRail(gamepad1.right_trigger-gamepad1.left_trigger, globalTimer);

                // if adjusting to low have the pitching preset go further in

                if (delay(100)){ // makes transfer better?
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE);
                    intakeSubsystem.intakeSpin(0);
                }
                if (delay(0)){ // run after we enter the new state
                    pitchTarget = (int)outtakeInverseKinematics.pitchEnd(pureHeight,0);
                    liftTarget = (int)outtakeInverseKinematics.slideEnd(pureHeight,0);
                    if (Math.abs(gamepad2.right_stick_y)<0.2){
                        fineAdjustHeight.upToggle(gamepad2.right_bumper || gamepad1.left_bumper);
                        fineAdjustHeight.downToggle(gamepad2.left_bumper || gamepad1.touchpad);
                        backdropRelativeHeight = fineAdjustHeight.OffsetTargetPosition;
                    } else {
                        fineAdjustHeight(gamepad2.right_stick_y); // this shouldn't conflict with bumpers because of rising edge detector logic
                        fineAdjustHeight.OffsetTargetPosition = (int)backdropRelativeHeight;
                    }
                    // this shoudl be rememebered because we are caching fineAdjustHeight
                }
                //RAIL_SERVO_POSITION += (int)outtakeInverseKinematics.railEnd(prevPureHeight,pureHeight,RAIL_SERVO_POSITION,headingPosition);

                // the line below tries to reduce the robot tipping from slides going out too fast
                outtakeSubsystem.liftToInternalPID(liftTarget, 1); //MaxExtension&&(backdropRelativeHeight<19)&&((liftTarget - ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition))>8)?0.7:

                if (ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) > LIFT_HITS_WHILE_PITCHING_THRESHOLD) { // so shit doesn't hit the thing when pitching
                    outtakeSubsystem.pitchToInternalPID(pitchTarget,1);
                } else{
                    outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
                }

                if ((gamepadRightBumperRisingEdge.mode(gamepad1.right_bumper)) || (delay(300) && reArrangePixels)) {
                    if (((liftTarget - ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition)) < 5) || reArrangePixels){
                        resetTimer();// reset timer
                        outtakeState = OuttakeState.DEPOSIT;
                        outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                        gamepad2.rumbleBlips(2);
                    }

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
                         if (topIsLeft){
                             outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.TOP_OPEN);
                         } else if (topIsRight){
                             outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.BOTTOM_OPEN);
                         }
                         droppedLeft = true;
                     }
                }

                if (droppedLeft && droppedRight){
                    outtakeState = OuttakeState.DEPOSIT;
                    resetTimer();// reset timer
                    gamepad2.rumbleBlips(2);
                }
                // drop logic


                if (delay(270)) { // ensure the arm is all the way out
                    //    if (!straightenTurret){
                    // telemetry.addLine("pointing to backdrop");
                    outtakeSubsystem.setOuttakeRailServo(RAIL_SERVO_POSITION); // RAIL_SERVO_POSITION SHOULD BE REMEMBERED
                    outtakeSubsystem.miniTurretPointToBackdrop(headingPosition);
                    setPivotServ();
                }

                break;

            case DEPOSIT:  // idk what this case does
                // if dropping with the slides further pitched than wait for longer lmao
                // or if we have dropped each pixel individually
                if (!reArrangePixels){
                    increaseHeight = true; // don't increase height if we have just rearranged pixels
                }

                /*
                if (outtakeSubsystem.pitchEncoderPosition > 48 || (droppedRight && droppedLeft)? delay() 500:delay() 150){ // test if manual reset is ok
                    if (!gamepad1.right_bumper){
                        outtakeState = OuttakeState.RETURN;
                        resetTimer();
                    }
                }
                 */
                outtakeSubsystem.miniTurretPointToBackdrop(headingPosition);
                if (delay(300)){
                    setPivotServ();
                    fineadJustStuff();
                }
                if (gamepadRightBumperRisingEdge.mode(gamepad1.right_bumper)){ // test if manual reset is ok
                    if (true){
                        outtakeState = OuttakeState.RETURN;
                        resetTimer();
                    }
                }
                break;

            case RETURN:
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
                intakeClipHoldorNotHold(-10,7);
                if (reArrangePixels){
                    RAIL_SERVO_POSITION = RAIL_CENTER_POS;
                }
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
                outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                intakeSubsystem.intakeSpin(-0.3);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);

                if (delay(100)){
                    outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
                    if (delay((Math.abs(RAIL_SERVO_POSITION-RAIL_CENTER_POS)*400) + 130)){
                        // the line above will retract the arm based on how far off the rail servo is at difference timing -> y = mx + c basically
                        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                        if (ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) < 3.5){ // lets slides ramming trick take over in ready state
                            outtakeState = OuttakeState.READY;
                            resetTimer();
                            //having these here means dont repeatedly set things so its more optimized
                            intakeSubsystem.intakeSpinState = IntakeSubsystem.IntakeSpinState.INTAKE;
                            changeExtension.ToggleMode = true;
                            usedTriggers = false;
                            droppedRight = false;
                            droppedLeft = false;
                            straightenTurret = false;
                            initialHeightStored = false; // resets this each cycle
                            initialPivot = true; // so that it pivots out left as default
                            reArrangePixels = false;
                            outtakeReset = false;

                            rightBumperPreExtend = false;
                            leftBumperPreExtend = false;
                            //RAIL_SERVO_POSITION = RAIL_CENTER_POS;

                            outtakeExtensionInches = MIN_OUTTAKE_EXTENSION_INCHES; // change if we want to extend more when not at max
                            if (increaseHeight){ // if it has been deposited
                                backdropRelativeHeight += FINE_ADJUST_HEIGHT_INTERVAL;
                            }
                            increaseHeight = false;
                            intakeOutBtnLogic.OffsetTargetPosition = 0;
                            //fineAdjustSlides.clearOffset(liftTarget); // delete this if you want the offset position to be remembered
                        }
                    }
                }
                if (pitchTarget > 36){
                    if (true){ // if we are pitching down wait a little bit
                        outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
                    }
                } else {
                    outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1);
                }

                if (outtakeSubsystem.pitchTargetReached()){
                    outtakeSubsystem.liftToInternalPID(0,1);
                } else {
                    outtakeSubsystem.liftToInternalPID(LIFT_HITS_WHILE_PITCHING_THRESHOLD, 1);
                }

                break;

            case MANUAL_ENCODER_RESET:  // idk what this case does
                outtakeSubsystem.liftMotorRawControl(gamepad2.right_stick_y + 0.35);
                //outtakeSubsystem.pitchMotorRawControl(gamepad2.left_stick_y);
                intakeSubsystem.intakeSlideMotorRawControl(gamepad2.left_trigger-gamepad2.right_trigger);
                outtakeSubsystem.setOuttakePitchPurplePixelPosition();
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE);
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
                    outtakeState = OuttakeState.READY;
                    resetTimer();
                }
                break;

            case CLIMB_START:
                outtakeSubsystem.setOuttakeRailServo(RAIL_CENTER_POS);
                if (delay(400)){
                    outtakeSubsystem.pitchTo(PITCH_CLIMB_TICKS, outtakeSubsystem.pitchEncoderPosition,1);
                    intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                    intakeSubsystem.intakeSlideInternalPID(100,1);
                } else {
                    outtakeSubsystem.liftToInternalPID(15,1);
                    outtakeSubsystem.pitchTo(PITCH_DEFAULT_DEGREE_TICKS,outtakeSubsystem.pitchEncoderPosition,1);
                }
                if (gamepad1.right_bumper){
                    outtakeState = OuttakeState.CLIMB_END;
                    resetTimer();
                }
                break;

            case CLIMB_END:  // idk what this case does
                outtakeSubsystem.pitchTo(16,outtakeSubsystem.pitchEncoderPosition, 1);
                outtakeSubsystem.liftToInternalPID(5,1); // not quite in
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                fineAdjustIntakeSlidesClimb(gamepad1.left_trigger - gamepad1.right_trigger);
                if (delay(100)){
                    if (intakeSubsystem.intakeSlideTargetReached()){ // slows the climb down a teeny tiny little bit
                        intakeSubsystem.intakeSlideInternalPID(550 + (int)intakeSlideAdjustTarget,1);
                    } else {
                        intakeSubsystem.intakeSlideInternalPID(550 + (int)intakeSlideAdjustTarget,0.4);
                    }
                }
                /*
                if (!climbAdjustIntakeSlides){
                    intakeSubsystem.intakeSlideInternalPID(300,1);
                    if (delay() 1000){
                        climbAdjustIntakeSlides = true;
                    }
                } else {
                    intakeSubsystem.intakeSlideMotorRawControl(gamepad1.left_trigger - gamepad1.right_trigger); // balances the robot out
                }
                 */
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

        if (gamepad2.left_stick_button && (outtakeState == OuttakeState.READY)){
            reArrangePixels = true;
            outtakeState = OuttakeState.OUTTAKE_ADJUST;
        }

        if (gamepad1.left_trigger > 0.2){ // && outtakeState == OuttakeState.READY
            if (gamepad1.dpad_up){
                driveBase.droneState(DriveBase.DroneServoState.RELEASE);
            }
            if (gamepad1.dpad_right){
                outtakeState = OuttakeState.CLIMB_START;
                resetTimer();
                //intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
                driveBase.droneState(DriveBase.DroneServoState.RELEASE);
                climbAdjustIntakeSlides = false;
            }
        }
    }

    public void setIntakeArmHeight(){ // Change this shit that lotus messed up
        if (gamepad2.a || gamepad1.a ){ //((outtakeState == OuttakeState.INTAKE || outtakeState == OuttakeState.INTAKE_EXTENDO) && intakeSubsystem.pixelsInIntake())
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
        } else if (gamepad2.x || gamepad1.x){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE);
        } else if (gamepad2.y || gamepad1.y){
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.TOP);
        }
    }

    public void intakeClipHoldorNotHold(int slideToPosition, int closeThreshold){
        if (intakeSubsystem.intakeSlidePosition < closeThreshold) {
            if (globalTimer - intakeClipTimer > 90){
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING); // turn the intake slide pid running to pos off to save battery draw
                intakeSubsystem.intakeSlideMotorRawControl(0);
            } else {
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING); // turn the intake slide pid running to pos off to save battery draw
//                intakeSubsystem.intakeSlideTo(slideToPosition,intakeSubsystem.intakeSlidePosition,1);
                intakeSubsystem.intakeSlideInternalPID(slideToPosition,1);
            }
        } else {
            intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // this might break something when as the intake slides won't go in, but stops jittering
            intakeSubsystem.intakeSlideInternalPID(slideToPosition,1);
          //  intakeSubsystem.intakeSlideTo(slideToPosition,intakeSubsystem.intakeSlidePosition,1);
            intakeClipTimer = globalTimer;
        }
    }
    public void intakeClipHoldorNotHoldWithoutPowerCutout(int slideToPosition){
        if (intakeSubsystem.intakeSlidePosition < 5) {
            if (globalTimer - intakeClipTimer > 100){
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING); // turn the intake slide pid running to pos off to save battery draw
                intakeSubsystem.intakeSlideInternalPID(0,1);
            } else {
                intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING); // turn the intake slide pid running to pos off to save battery draw
                intakeSubsystem.intakeSlideInternalPID(slideToPosition,1);
               // intakeSubsystem.intakeSlideTo(slideToPosition,intakeSubsystem.intakeSlidePosition,1);
            }
        } else {
            intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // this might break something when as the intake slides won't go in, but stops jittering
            intakeSubsystem.intakeSlideInternalPID(slideToPosition,1);
           // intakeSubsystem.intakeSlideTo(slideToPosition,intakeSubsystem.intakeSlidePosition,1);
            intakeClipTimer = globalTimer;
        }
    }

    public void adjustRail(){
        if (gamepad1.dpad_down){
           // outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
            RAIL_SERVO_POSITION = RAIL_CENTER_POS;
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
        outtakeSubsystem.pitchToInternalPID(PITCH_DEFAULT_DEGREE_TICKS,1); // internalPID uses less power draw - integral term is better
        if (gamepad2.left_trigger > 0.2 && gamepad2.left_bumper) { // must hold these 2 things and the slides will slam up and down - dodge fix
            outtakeSubsystem.resetOuttake();
            outtakeSubsystem.outtakeResetState = OuttakeSubsystem.OuttakeResetState.UP; // starts the thing
        } else {
            if (ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) < 0.2){
                outtakeSubsystem.liftToInternalPID(0,1);
                outtakeReset = true;

                //outtakeSubsystem.liftMotorRawControl(0);
            } else if(!outtakeReset){
                outtakeSubsystem.liftTo(-100,outtakeSubsystem.liftPosition,1);
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
            liftTarget = LIFT_INCHES_FOR_MAX_EXTENSION;
            pitchTarget = (int)Math.toDegrees(Math.asin(pureHeight/LIFT_INCHES_FOR_MAX_EXTENSION));
            if (pitchTarget > 60){
                pitchTarget = 60;
            } else if (pitchTarget < 20){
                pitchTarget = 20;
            }
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
        fineAdjustHeight.clearOffset((int)backdropRelativeHeight);

    }
    // This function isn't run in ready state
    public void liftPositionChange(boolean D1controls, boolean afterExtended){ // if gamepad inputs don't work in this class then pass them through as parameters in the function
        if (highPreset.mode(gamepad2.y || gamepad1.y && D1controls)){ // everything in here should only run once
            adjustHeightCache = HIGH_BACKDROP_PRESET_INCHES;
            initialHeightStored = false;
            adjustingToMajorPresetLogic(afterExtended); // this should work for all cases - very complex and will require alot of telemetry
        } else if (midPreset.mode(gamepad2.x || gamepad1.x) && D1controls){
            adjustHeightCache = MID_BACKDROP_PRESET_INCHES; // this part can be changed in throughout any part of the program
            initialHeightStored = false;
            adjustingToMajorPresetLogic(afterExtended);
        } else if (lowPreset.mode(gamepad2.a || gamepad1.a && D1controls) ){
            adjustHeightCache = LOW_BACKDROP_PRESET_INCHES;
            initialHeightStored = false;
            adjustingToMajorPresetLogic(afterExtended);
        }
    }
    public void adjustingToMajorPresetLogic(boolean afterExtended){
        if (afterExtended && !initialHeightStored){
            initialHeightStored = true; // this is reset every cycle
            if (backdropRelativeHeight - adjustHeightCache > 0){ // this means that we are adjusting down
                MaxExtension = false; // best way to explain this is that as soon as the height is selected the outtake extension limit will also be calculated
                outtakeExtensionInches = outtakeExtensionInches - ((backdropRelativeHeight - adjustHeightCache)/Math.tan(Math.toRadians(60)));
                if (outtakeExtensionInches < MIN_OUTTAKE_EXTENSION_INCHES){ // clip outtake extension range :)
                    outtakeExtensionInches = MIN_OUTTAKE_EXTENSION_INCHES;
                }
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
        if (gamepad2.dpad_down || (outtakeState == OuttakeState.DEPOSIT && reArrangePixels)){
            pivotState = 6; // this case will get hit first
        } else if (gamepad2.left_trigger > 0.2) {
            if (pivotFlipToggle.ToggleMode){
                pivotState = 0;
                topIsRight = true;
                topIsLeft = false;
            } else {
                pivotState = 1;
                topIsLeft = true;
                topIsRight = false;
            }
        } else if (gamepad2.right_trigger > 0.2){
            if (pivotFlipToggle.ToggleMode){
                pivotState = 2;
                topIsLeft = true;
                topIsRight = false;
            } else {
                pivotState = 3;
                topIsRight = true;
                topIsLeft = false;
            }
        } else if (gamepad2.dpad_left || initialPivot){
            pivotState = 4;
            topIsLeft = true;
            topIsRight = false;
            initialPivot = false;
        } else if (gamepad2.dpad_right){
            pivotState = 5;
            topIsRight = true;
            topIsLeft = false;
        }
    }

    public void setPivotServ(){
        if (pivotState == 0){
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_LEFT_FLIPPED);
        } else if (pivotState == 1){
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_LEFT);
        } else if (pivotState == 2){
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_RIGHT_FLIPPED);
        } else if (pivotState == 3){
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_RIGHT);
        } else if (pivotState == 4){
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
        } else if (pivotState == 5){
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
        } else if (pivotState == 6){
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
        }
    }

    public void fineAdjustHeight(double fineAdjust){
        if (globalTimer - heightAdjustTimer > 15){ // only set the position every 15 ms, once achieved cache the timer value
            backdropRelativeHeight += -fineAdjust * 0.35;
            heightAdjustTimer = globalTimer; // cache the value of the outtakerailadjust
        }
    }
    public void fineAdjustIntakeSlidesClimb(double fineAdjust){
        if (globalTimer - heightAdjustTimer > 15){
            intakeSlideAdjustTarget += fineAdjust * 4;
            intakeSlideAdjustTimer = globalTimer;
        }
    }

    public void fineadJustStuff(){
        pitchTarget = (int)outtakeInverseKinematics.pitchEnd(pureHeight,0);
        liftTarget = (int)outtakeInverseKinematics.slideEnd(pureHeight,0);
        if (Math.abs(gamepad2.right_stick_y)<0.2){
            fineAdjustHeight.upToggle(gamepad2.right_bumper || gamepad1.left_bumper);
            fineAdjustHeight.downToggle(gamepad2.left_bumper || gamepad1.touchpad);
            backdropRelativeHeight = fineAdjustHeight.OffsetTargetPosition;
        } else {
            fineAdjustHeight(gamepad2.right_stick_y); // this shouldn't conflict with bumpers because of rising edge detector logic
            fineAdjustHeight.OffsetTargetPosition = (int)backdropRelativeHeight;
        }
        outtakeSubsystem.pitchToInternalPID(pitchTarget,1);
        outtakeSubsystem.liftToInternalPID(liftTarget,1);
        outtakeSubsystem.fineAdjustRail(-gamepad2.right_stick_x * 0.7 + (gamepad1.right_trigger-gamepad1.left_trigger), globalTimer);
        outtakeSubsystem.setOuttakeRailServo(RAIL_SERVO_POSITION); // RAIL_SERVO_POSITION SHOULD BE REMEMBERED
        //RAIL_SERVO_POSITION += outtakeInverseKinematics.railEndTicks(prevPureHeight,pureHeight,RAIL_SERVO_POSITION,headingPosition);//RAIL_SERVO_POSITION += (int)outtakeInverseKinematics.railEnd(prevPureHeight,pureHeight,RAIL_SERVO_POSITION,headingPosition);
    }

    public void liftDown(double liftTheshold){
        if (ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) < liftTheshold){ // if the lift is where we want it
            outtakeSubsystem.liftToInternalPID(-0.15,1);
        } else {
            outtakeSubsystem.liftToInternalPID(-3,1);
        }
    }

    public void bumperPressed(){
        if (gamepad1.right_bumper){
            rightBumperPreExtend = true;
        } else if (gamepad1.left_bumper){
            leftBumperPreExtend = true;
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

    public double gamepad2RightStickXWithDeadzone(){
        if (Math.abs(gamepad2.right_stick_x) < 0.16){
            return 0;
        } else{
            return gamepad2.right_stick_x;
        }
    }

    public boolean delay(double delayTime){
        return globalTimer - sequenceTimer > delayTime;
    }

    public void resetTimer(){
        sequenceTimer = globalTimer;
    }



}



