package org.firstinspires.ftc.teamcode.system.accessory.profile;

public class ProfileState {
    public double x = 0;
    public double v = 0;
    public double a = 0;

    public ProfileState(double x, double v, double a) {
        this.x = x;//this.x = x;
        this.v = v;
        this.a = a;
    }
// I am keeping this as a separate class if we create different profiles
    public ProfileState() {}
}