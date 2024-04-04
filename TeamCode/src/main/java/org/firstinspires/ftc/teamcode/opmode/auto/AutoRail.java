package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.RAIL_CENTER_YELLOW_POS;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.S;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.numCycles;
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
            } else if (trussMiddleStage == 1){
                if (teamPropLocation == 2){
                    //auto.outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER_YELLOW_TRUSS);
                    //if (auto.cameraHardware.getPreloadYellowPose() == Globals.Place.LEFT){

                    //}
                    auto.cameraHardware.setDefaultRailPos(RAIL_CENTER_YELLOW_POS);
                    auto.outtakeSubsystem.setOuttakeRailServo(railAprilTagTarget);
                } else if (teamPropLocation == 1){
                    auto.outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.LEFT);
                } else if (teamPropLocation == 3){
                    auto.outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.RIGHT);
                }
            } else if (trussMiddleStage == 3){
                if (teamPropLocation == 2){
                    auto.outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.CENTER_YELLOW_STAGE);
                } else if (teamPropLocation == 1){
                    auto.outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.LEFT_YELLOW_STAGE);
                } else if (teamPropLocation == 3){
                    auto.outtakeSubsystem.outtakeRailState(OuttakeSubsystem.OuttakeRailState.RIGHT_YELLOW_STAGE);
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
}
