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
                ToggleMode = !ToggleMode;
                Toggled = true;
            }
        }
        else {
            Toggled = false;
        }
    }
}
