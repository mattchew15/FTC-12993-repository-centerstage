package org.firstinspires.ftc.teamcode.system.accessory;

public class ToggleR
{
    private boolean toggleMode, toggled;
    public ToggleR()
    {
        this.toggleMode = false;
        this.toggled = false;
    }

    public boolean mode(boolean btn)
    {
        if (btn)
        {
            if (!toggled)
            {
                toggleMode = !toggleMode;
                toggled = true;
            }
        }
        else
        {
            toggled = false;
        }
        return toggleMode;
    }
}
