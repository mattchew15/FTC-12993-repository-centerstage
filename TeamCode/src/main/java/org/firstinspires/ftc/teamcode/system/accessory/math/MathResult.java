package org.firstinspires.ftc.teamcode.system.accessory.math;

public class MathResult {
    double prevInput;
    double result;

    public MathResult(double prevInput, double result) {
        this.prevInput = prevInput;
        this.result = result;
    }

    public double getPrevInput()
    {
        return prevInput;
    }

    public double getResult()
    {
        return result;
    }
}
