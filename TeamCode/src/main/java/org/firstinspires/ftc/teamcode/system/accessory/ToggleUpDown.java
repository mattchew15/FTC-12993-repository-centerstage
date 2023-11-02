package org.firstinspires.ftc.teamcode.system.accessory;

public class ToggleUpDown {

    public boolean CycleDown;
    public boolean CycleUp;
    public int CycleState;
    public int CycleLimit;

    public ToggleUpDown(int cycleLimit){ // constructor means we don't need to run additional function
        CycleDown = false;
        CycleUp = false;
        CycleState = 0;
        this.CycleLimit = cycleLimit;

    }

    public void cycleToggleUpDown(boolean cyclebtnup, boolean cyclebtndown){
        if (cyclebtnup) {
            if (!CycleUp) {
                CycleUp = true;
                CycleState += 1;
            }
        }
        else {
            CycleUp = false;
        }
        if (cyclebtndown) {
            if (!CycleDown) {
                CycleDown = true;
                CycleState -= 1;
            }
        }
        else {
            CycleDown = false;
        }
        if (CycleState < 0){
            CycleState = 0;
        }
        if (CycleState > CycleLimit){
            CycleState = CycleLimit;
        }
    }
}
