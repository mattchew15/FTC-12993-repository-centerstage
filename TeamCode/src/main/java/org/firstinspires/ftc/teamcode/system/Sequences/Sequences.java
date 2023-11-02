package org.firstinspires.ftc.teamcode.system.Sequences;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

public class Sequences {
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    ElapsedTime sequenceTimer;
    double transferGrabTimer;

    public Sequences(IntakeSubsystem intakeSubsystem, OuttakeSubsystem outtakeSubsystem, ElapsedTime timer){
        this.sequenceTimer = timer;
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem; // this allows the declared subsystem to be associated with the ones passed into the constructor - if i don't do this i get a null error
    }

    public void setupSequences(){ // run this with setup functions
        transferGrabState = TransferGrabState.IDLE;
    }

    enum TransferGrabState { // could make these private?
        PIXEL_GRAB,
        FLAP_OPEN,
        ARM_PRE_EXTEND,
        IDLE
    }

    TransferGrabState transferGrabState;

   public void transferGrab(boolean dropBtn){ // pass in timer as a parameter??
        switch (transferGrabState){
            case PIXEL_GRAB:
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.TRANSFER);
                outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.CLOSE);
                intakeSubsystem.intakeSpin(0.5);
                if (sequenceTimer.milliseconds() - transferGrabTimer > 200){
                    transferGrabState = TransferGrabState.FLAP_OPEN;
                    transferGrabTimer = sequenceTimer.milliseconds(); // resets timer
                }
                break;
            case FLAP_OPEN:
                intakeSubsystem.intakeFlapServoState(IntakeSubsystem.IntakeFlapServoState.OPEN);
                outtakeSubsystem.wristServoState(OuttakeSubsystem.WristServoState.TRANSFER);
                if (sequenceTimer.milliseconds() - transferGrabTimer > 90){
                    transferGrabState = TransferGrabState.ARM_PRE_EXTEND;
                }
                break;
            case ARM_PRE_EXTEND:
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.SCORE_UP);
                outtakeSubsystem.wristServoState(OuttakeSubsystem.WristServoState.SCORE);
                if (dropBtn){
                    dropPixels();
                }
                break;

            case IDLE:
                outtakeSubsystem.armServoState(OuttakeSubsystem.ArmServoState.READY);
                intakeSubsystem.intakeFlapServoState(IntakeSubsystem.IntakeFlapServoState.CLOSE);
                break;
        }
    }

    public void startTransfer(){
       transferGrabState = TransferGrabState.PIXEL_GRAB;
        transferGrabTimer = sequenceTimer.milliseconds(); // resets timer
    }


    public void dropPixels(){
        outtakeSubsystem.clawServoState(OuttakeSubsystem.ClawServoState.OPEN);
    }

    private enum TransferState {

    }
    private enum OuttakeState {

    }
}
