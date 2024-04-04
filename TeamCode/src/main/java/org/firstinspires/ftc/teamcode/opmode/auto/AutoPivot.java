package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.BLUE_AUTO;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RED_AUTO;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.numCycles;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.teamPropLocation;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

public class AutoPivot {
    AutoSequences auto;
    int yellowCycle;
    int numCyclesForStraightPivot;
    int trussMiddleStage;
    Telemetry telemetry;

    public AutoPivot(int numCyclesForStraightPivot, int yellowCycle, AutoSequences auto, Telemetry telemetry, int trussMiddleStage){
        this.numCyclesForStraightPivot = numCyclesForStraightPivot;
        this.yellowCycle = yellowCycle;
        this.auto = auto;
        this.telemetry = telemetry;
        this.trussMiddleStage = trussMiddleStage;
    }

    public void pivotLogic (){
        if (numCycles == yellowCycle){
            //telemetry.addLine("numCycles == yellowCycle)");
            if (trussMiddleStage == 2){
                if (teamPropLocation == 1){
                    if (BLUE_AUTO){
                        auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
                    }
                    else {
                        auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
                    }
                } else if (teamPropLocation == 2){
                    if (RED_AUTO){
                        auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_RIGHT_FLIPPED);
                    }
                    else {
                        auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_LEFT_FLIPPED);
                    }
                } else if (teamPropLocation == 3){
                    if (RED_AUTO){
                        auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
                    }
                    else {
                        auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
                    }
                }
            } else if (trussMiddleStage == 1){ // cos they have already placed!!!
                auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_RIGHT_FLIPPED);
            } else if (trussMiddleStage == 3){
                if (teamPropLocation == 1){
                    if (RED_AUTO){
                        auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_RIGHT_FLIPPED);
                    }
                    else {
                        auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_LEFT_FLIPPED);
                    }
                } else if (teamPropLocation == 2){
                    if (RED_AUTO){
                        auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_LEFT_FLIPPED);
                    }
                    else {
                        auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_RIGHT_FLIPPED);
                    }
                } else if (teamPropLocation == 3){
                    if (RED_AUTO){
                        auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_LEFT_FLIPPED);
                    }
                    else {
                        auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.DIAGONAL_RIGHT_FLIPPED);
                    }
                }

            }

        } else if (numCycles > numCyclesForStraightPivot){
            auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.READY);

        } else {
            if (BLUE_AUTO){
                auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_LEFT);
            }
            else {
                auto.outtakeSubsystem.pivotServoState(OuttakeSubsystem.PivotServoState.SIDEWAYS_RIGHT);
            }
        }
    }
}
