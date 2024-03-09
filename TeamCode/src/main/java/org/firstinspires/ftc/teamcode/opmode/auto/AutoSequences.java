/*

package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.MiddleLaneYDeposit;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.poseEstimate;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.xPosition;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.yPosition;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@Config
public class AutoSequences {

    ElapsedTime GlobalTimer;
    double autoTimer;

    int numCycles;
    boolean changeStates;
    int outtakeChangeStates;



    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    AutoTrajectories autoTrajectories = new AutoTrajectories(); // road drive class

    public void initAutoHardware (HardwareMap hardwareMap){
        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        autoTrajectories.init(hardwareMap);
    }

    public void intializationLoop (){
        outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
        intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
    }

    public void afterWaitForStart(){
        GlobalTimer = new ElapsedTime(System.nanoTime());
        GlobalTimer.reset();
        autoTimer = GlobalTimer.milliseconds();

        intakeSubsystem.intakeHardwareSetup();
        outtakeSubsystem.hardwareSetup();
        outtakeSubsystem.encodersReset();
        intakeSubsystem.intakeSlideMotorEncodersReset();

        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop
        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY); // don't touch this at all in auto
        autoTimer = 0;
        numCycles = 0;
        outtakeSubsystem.outtakeDistanceSensorValue = 100; // so that it doesn't do funky stuff
        autoTrajectories.drive.setPoseEstimate(autoTrajectories.startPoseFront);
    }

    public void delayState(double delaytime){
        outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.UPRIGHT);
        outtakeSubsystem.pitchToInternalPID(680,1);
        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN);
        if (GlobalTimer.milliseconds() - autoTimer > delaytime){
            autoTimer = GlobalTimer.milliseconds(); // reset timer not rly needed here
            changeStates = true;
            if (teamPropLocation == 1){
                autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive1Front);
            } else if (teamPropLocation == 2){
                autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive2Front);

            } else if (teamPropLocation == 3){
                autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.PreloadDrive3Front);

            }
        }
    }
    public void preloadDriveState (double timeBeforeArmGoesToGround){
        intakeSubsystem.intakeSlideInternalPID(0,1);
        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);
        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.OPEN); // just so we don't have an extra write during the loop

        if (GlobalTimer.milliseconds() - autoTimer > timeBeforeArmGoesToGround){
            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_UP);
        }
        if (!autoTrajectories.drive.isBusy()){
            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);

            outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
            changeStates = true;
            autoTimer = GlobalTimer.milliseconds();
            intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
        }

    }

    public void placeAndIntake (){
        intakeSubsystem.intakeSpin(1);
        if (teamPropLocation == 1){
            intakeSubsystem.intakeSlideTo(100, intakeSubsystem.intakeSlidePosition,1);
        } else if (teamPropLocation == 2){
            intakeSubsystem.intakeSlideTo(295, intakeSubsystem.intakeSlidePosition,1);
        } else if (teamPropLocation == 3){
            intakeSubsystem.intakeSlideTo(480, intakeSubsystem.intakeSlidePosition,1);
        }
        if (intakeSubsystem.intakeSlideTargetReached()){
            if (GlobalTimer.milliseconds() - autoTimer > 700){ // ensure pixels are in robot
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.HOLDING);
                changeStates = true;
                if (teamPropLocation == 1){
                    autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.AfterPreloadDrive1Front);
                } else if (teamPropLocation == 2){
                    autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.AfterPreloadDrive2Front);
                } else if (teamPropLocation == 3){
                    autoTrajectories.drive.followTrajectoryAsync(autoTrajectories.AfterPreloadDrive3Front);
                }
            }
        } else {
            autoTimer = GlobalTimer.milliseconds(); // spams timer reset - sneaky trick
            intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.OPEN);
        }
        outtakeSubsystem.pitchToInternalPID(SIXTY_DEGREE_TICKS,1);
        if (GlobalTimer.milliseconds() - autoTimer > 150){
            outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
        }
    }

    public void afterPurpleDrive(){
        intakeSubsystem.intakeSlideInternalPID(0,1);
        intakeSubsystem.intakeSpin(-1);
        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP);
        if (yPosition < -27.5){
            // autoTrajectories.outtakeDriveMiddlePath(poseEstimate, 20);
            autoTrajectories.outtakeDriveMiddlePath(poseEstimate,20, 30, -30);
            autoTimer = GlobalTimer.milliseconds();
            changeStates = true;
        }
    }

    public void transferPixel(){
        intakeSubsystem.intakeSlideInternalPID(-3,1);
        if (GlobalTimer.milliseconds() - autoTimer > 170){ // time for pixel holder to close
            if (GlobalTimer.milliseconds() - autoTimer > 350){
                intakeSubsystem.intakeSpin(0.5);
                if (intakeSubsystem.intakeSlidePosition < 5){
                    intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.TRANSFER);
                    if (intakeSubsystem.intakeSlidePosition < 2){
                        changeStates = true;
                        autoTimer = GlobalTimer.milliseconds();
                    }
                } else {
                    intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.HALF_UP);
                }
            } else {
                intakeSubsystem.intakeSpin(-1);
            }
        } else {
            intakeSubsystem.intakeSpin(0.5); // this doesn't happen for very long
        }
    }

    public void outtakePixel(double correctedHeading, Telemetry telemetry){
        if (GlobalTimer.milliseconds() - autoTimer > 550 ){ // wait for chute to go in properly
            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
            if (GlobalTimer.milliseconds() - autoTimer > 650){ // time for grippers to close
                intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);

                if (numCycles == 0){
                    outtakeSubsystem.pitchToInternalPID(1220,1);
                    outtakeSubsystem.updateLiftTargetProfile(570);
                    //outtakeSubsystem.liftTo(699,outtakeSubsystem.liftPosition,1);
                } else if (numCycles == 1){
                    //outtakeSubsystem.liftTo(725,outtakeSubsystem.liftPosition,1);
                    outtakeSubsystem.updateLiftTargetProfile(690);
                    outtakeSubsystem.pitchToInternalPID(980,1);
                } else if (numCycles == 2){
                    outtakeSubsystem.updateLiftTargetProfile(729);
                    //outtakeSubsystem.liftTo(738,outtakeSubsystem.liftPosition,1);
                    outtakeSubsystem.pitchToInternalPID(840,1);
                } else if (numCycles == 3){
                    outtakeSubsystem.updateLiftTargetProfile(738);
                    //outtakeSubsystem.liftTo(738,outtakeSubsystem.liftPosition,1);
                    outtakeSubsystem.pitchToInternalPID(900,1);
                }
                outtakeSubsystem.profileLiftCalculate();
                if (GlobalTimer.milliseconds() - autoTimer > 950){
                    outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_DOWN); // slides go out before arm so transfer is good
                    if (GlobalTimer.milliseconds() - autoTimer > 1150){ // once chute is down
                        outtakeSubsystem.miniTurretPointToBackdrop(correctedHeading);
                        if (numCycles == 0){
                            if (teamPropLocation == 1){
                                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
                            } else if (teamPropLocation == 2){
                                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
                            } else if (teamPropLocation == 3){
                                outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
                            }
                        } else {
                            outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
                        }
                        if (numCycles < 2){
                            intakeSubsystem.intakeSlideTo(250, intakeSubsystem.intakeSlidePosition, 1); // line above may limit speed of drop
                        } else {
                            intakeSubsystem.intakeSlideTo(0, intakeSubsystem.intakeSlidePosition, 1); // line above may limit speed of drop
                        }
                        telemetry.addLine("DRIVE IS SITLL BUSY!!!!!");
                        if (!autoTrajectories.drive.isBusy()) { // // || xPosition > 35 this could make it faster on the first cycle
                            // line above determines when we drop the pixels
                            outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                            numCycles += 1;
                            autoTimer = GlobalTimer.milliseconds();

                            if (numCycles == 1){
                                autoTrajectories.driveIntoStackStraight(poseEstimate,12,2); // might cause an issue here
                            } else if (numCycles == 2){
                                autoTrajectories.driveIntoStackStraight(poseEstimate,18,2); // might cause an issue here
                                outtakeChangeStates = 1;
                            } else if (numCycles == 3){
                                autoTrajectories.driveIntoStackStageFromMiddlePathStraightEnd(poseEstimate,20,30);
                                outtakeChangeStates = 1;
                            } else if (numCycles == 4){
                                outtakeChangeStates = 2;
                                autoTrajectories.park(poseEstimate,2);
                            }
                        }
                    }
                }
            }
        }
    }

    public void drop(){
        // waits for robot to move away far enough - minimize this
        if (GlobalTimer.milliseconds() - autoTimer > 100){
            outtakeSubsystem.liftToInternalPID(-6,1);
            outtakeSubsystem.pitchToInternalPID(SIXTY_DEGREE_TICKS,1); // SIXTY_DEGREE_TICKS = 0 lmao
            if (GlobalTimer.milliseconds() - autoTimer > 200){ // not knock pixels off the backdrop
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
                if (numCycles == 1){
                    intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
                } else if (numCycles == 2){
                    intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE);
                } else if (numCycles == 3){
                    // make intake arm height correct
                    intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.VERY_TOP);
                } else if (numCycles == 4) {
                    intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.MIDDLE);
                }
                changeStates = true;
                autoTimer = GlobalTimer.milliseconds();
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.OPEN);

            }

        }
    }

    public void grabOffStack(){
        outtakeSubsystem.pitchToInternalPID(SIXTY_DEGREE_TICKS, 1);
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
        outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);
        outtakeSubsystem.liftToInternalPID(0,1);
        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
        intakeSubsystem.intakeChuteArmServoState(IntakeSubsystem.IntakeChuteServoState.READY);


        if (xPosition < 0){ //
            intakeSubsystem.intakeSlideTo(700, intakeSubsystem.intakeSlidePosition,1);
            if (xPosition < -12) {
                intakeSubsystem.intakeSpin(1);
                if (xPosition < -35.2){ // do stuff with sensor to make better

                    changeStates = true;
                    autoTimer = GlobalTimer.milliseconds();
                    if (numCycles > 2) {
                        autoTrajectories.outtakeDriveTurnEndPath(poseEstimate,22,150, 29,3);
                    } else {
                        autoTrajectories.outtakeDriveMiddlePath(poseEstimate,22, 25.5, MiddleLaneYDeposit);
                    }
                }
            }
        }
    }

    public void afterGrabOffStack(){
          /*
                if (intakeSubsystem.pixelsInIntake()){
                    intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.HOLDING);
                }


        if (GlobalTimer.milliseconds() - autoTimer > 100){
            if (numCycles == 1 || numCycles == 3) {
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.FOUR);
            } else if (numCycles == 2){
                intakeSubsystem.intakeArmServoState(IntakeSubsystem.IntakeArmServoState.BASE);
            }
            if (GlobalTimer.milliseconds() - autoTimer > 300){
                autoTimer = GlobalTimer.milliseconds(); // resets timer
                changeStates = true;
                intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderState.HOLDING);
            }
        }
    }

    public void park(){
        if (GlobalTimer.milliseconds() - autoTimer > 200){
            changeStates = true;
        }
    }

    public void idle(Telemetry telemetry){
        telemetry.addLine("WWWWWWWWWWW");
        intakeSubsystem.intakeSlideInternalPID(-6,1);
        outtakeSubsystem.liftToInternalPID(-8,1);
        outtakeSubsystem.pitchToInternalPID(SIXTY_DEGREE_TICKS, 1);
        outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
        outtakeSubsystem.miniTurretState(OuttakeSubsystem.MiniTurretState.STRAIGHT);
    }

}


 */
