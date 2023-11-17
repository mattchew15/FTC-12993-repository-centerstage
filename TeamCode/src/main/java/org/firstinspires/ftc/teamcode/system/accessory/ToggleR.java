package org.firstinspires.ftc.teamcode.system.accessory;

public class ToggleR
{
    private boolean toggleMode;
    public ToggleR()
    {
        this.toggleMode = false;
    }

    public boolean mode(boolean btn)
    {
        if (btn)
        {
            if (!toggleMode)
            {
                toggleMode = true;
                return true;
            }
        }
        else
        {
            toggleMode = false;
        }
        return false;
    }
}
