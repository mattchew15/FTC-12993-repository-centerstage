package org.firstinspires.ftc.teamcode.system.accessory;

public class ToggleR
{
    private boolean toggle, toggleMode;
    public ToggleR()
    {
        this.toggle = false;
        this.toggleMode = false;
    }

    public boolean mode(boolean btn)
    {
        if (btn)
        {
            if (!toggleMode)
            {
                toggleMode = true;
                toggle = true;
            }
            else
            {
                toggle = false;
            }
        }
        else
        {
            toggle = false;
            toggleMode = false;
        }
        return toggle;

    }
}
