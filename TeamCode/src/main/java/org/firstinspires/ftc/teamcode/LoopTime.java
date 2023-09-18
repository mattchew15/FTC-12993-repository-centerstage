package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LoopTime {
    double dt;
    double prev_time;

    public void updateLoopTime (Telemetry telemetry){
        dt = System.currentTimeMillis() - prev_time;
        telemetry.addData("Loop Time", dt);
        prev_time = System.currentTimeMillis(); // this should possibly be run at the end of the loop to reset the time, might not work all packaged together
    }

}
