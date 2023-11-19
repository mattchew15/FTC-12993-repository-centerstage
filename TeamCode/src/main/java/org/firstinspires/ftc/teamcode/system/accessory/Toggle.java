package org.firstinspires.ftc.teamcode.system.accessory;

public class Toggle {

    public boolean Toggled;
    public boolean ToggleMode;

    public Toggle (){
        Toggled = false;
        ToggleMode = false;
    }

    public void ToggleMode(boolean togglebtn) {
        if (togglebtn) {
            if (!Toggled) { // the first time you first press it it will change stuff, then won't go past this if statement
                if (ToggleMode) {
                    ToggleMode = false; // will have to access this variable in dune drive
                } else {
                    ToggleMode = true;
                }
                Toggled = true;
            }
        }
        else {
            Toggled = false;
        }
    }
}
