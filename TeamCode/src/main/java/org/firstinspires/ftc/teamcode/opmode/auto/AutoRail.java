package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_CENTER_YELLOW_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_CENTER_YELLOW_STAGE_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_CENTER_YELLOW_TRUSS_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_LEFT_YELLOW_STAGE_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_LEFT_YELLOW_TRUSS_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_RIGHT_YELLOW_STAGE_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_RIGHT_YELLOW_TRUSS_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.S;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.inchesToTicksRailCorrected;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.numCycles;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.place;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.teamPropLocation;

import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

public class AutoRail {
    //AutoSequences auto = new AutoSequences();

    int numCyclesForExtendedRail;
    int yellowCycle;
    int trussMiddleStage;
    boolean railGoesRight;
    double railAprilTagTarget;

    final double pivotOffset = 0.6;

    AutoSequences auto;

    public AutoRail(int numCyclesForExtendedRail, int yellowCycle, AutoSequences auto, boolean railGoesRight, int trussMiddleStage){
        this.numCyclesForExtendedRail = numCyclesForExtendedRail;
        this.yellowCycle = yellowCycle;
        this.auto = auto;
        this.railGoesRight = railGoesRight;
        this.trussMiddleStage = trussMiddleStage;
    }

    public void setRailTargetFromAprilTag(double railTarget){
        railAprilTagTarget = railTarget;
    }

    public void railLogic(){
        if (numCycles == yellowCycle){ //TODO mirror the first yellow placement
            if (trussMiddleStage == 2){
                if (teamPropLocation == 2){
                    auto.outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER_YELLOW);
                } else if (teamPropLocation == 1){
                    auto.outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.LEFT);
                } else if (teamPropLocation == 3){
                    auto.outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.RIGHT);
                }
            } else if (trussMiddleStage == 3){
                if (teamPropLocation == 2){
                    setAprilTagRailThingy(RAIL_CENTER_YELLOW_STAGE_POS,-pivotOffset,-pivotOffset);
                } else if (teamPropLocation == 1){
                    setAprilTagRailThingy(RAIL_LEFT_YELLOW_STAGE_POS,pivotOffset,pivotOffset);
                } else if (teamPropLocation == 3){
                    setAprilTagRailThingy(RAIL_RIGHT_YELLOW_STAGE_POS,-pivotOffset,-pivotOffset);
                }
            } else if (trussMiddleStage == 1){
                if (teamPropLocation == 2){
                    // the pivot is the same as stage side just different for center case
                    setAprilTagRailThingy(RAIL_CENTER_YELLOW_STAGE_POS,pivotOffset,pivotOffset);
                } else if (teamPropLocation == 1){
                    setAprilTagRailThingy(RAIL_LEFT_YELLOW_STAGE_POS,pivotOffset,pivotOffset);
                } else if (teamPropLocation == 3){
                    setAprilTagRailThingy(RAIL_RIGHT_YELLOW_STAGE_POS,-pivotOffset,-pivotOffset);
                }
            }

        } else if (numCycles > numCyclesForExtendedRail){
            if (S == 1){ // mirrored cases for auto
                if (railGoesRight){
                    auto.outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.RIGHT);
                } else {
                    auto.outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.LEFT);
                }

            } else{
                if (railGoesRight){
                    auto.outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.LEFT);
                } else {
                    auto.outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.RIGHT);
                }
            }
        } else {
            auto.outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER);
        }
    }
    public void setAprilTagRailThingy(double defaultPosition, double pivotOffsetRight, double pivotOffsetLeft){
        auto.cameraHardware.setDefaultRailPos(defaultPosition);
        double railOffset = 0;
        if (place == Globals.Place.RIGHT){
            railOffset = inchesToTicksRailCorrected(pivotOffsetRight);
        } else if (place == Globals.Place.LEFT){
            railOffset = inchesToTicksRailCorrected(pivotOffsetLeft);
        }
        auto.outtakeSubsystem.setOuttakeRailServo(railAprilTagTarget + railOffset);
    }
}
