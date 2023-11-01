package org.firstinspires.ftc.teamcode.system.Sequences;

import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

public class TeleopSequence {
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;

    public void things(){
        outtakeSubsystem.liftTo(1,outtakeSubsystem.liftPosition,1);
        outtakeSubsystem.pitchTo(1,outtakeSubsystem.pitchPosition,1);
    }
    enum PixelDepositState { // could make these private?
        READY,
        CONE_DROP,
        BRACE_RETRACT,
        ARM_RESET,
        IDLE
    }
    private enum TransferState {

    }
    private enum OuttakeState {

    }
}
