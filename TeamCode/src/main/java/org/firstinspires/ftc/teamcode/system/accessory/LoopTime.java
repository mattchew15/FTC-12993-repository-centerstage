package org.firstinspires.ftc.teamcode.system.accessory;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LoopTime { // Do you need to cache 2 variables instead of just time.reset()
    double dt;
    double prev_time;
    Telemetry telemetry;
    public LoopTime(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }
    public LoopTime()
    {

    }

    public void dt()
    {
        // the other one would make them shorter or maybe longer because there is a telemetry call in the middle
        double now = System.currentTimeMillis();
        dt = now - prev_time;
        prev_time = now;
    }

    public void updateLoopTime (@NonNull Telemetry telemetry){

        dt = System.currentTimeMillis() - prev_time;
        telemetry.addData("Loop Time", dt);
        prev_time = System.currentTimeMillis(); // this should possibly be run at the end of the loop to reset the time, might not work all packaged together
    }
    public double getDt()
    {
        return dt;
    }
}
