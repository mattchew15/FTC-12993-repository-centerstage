package org.firstinspires.ftc.teamcode.system.accessory.profile;

public class ProfileConstraints {
    public double accel;
    public double decel;
    public double velo;

    // if max accel and decel are the same
    public ProfileConstraints(double velo, double accel) {
        this(accel, accel, velo);
    }

    public ProfileConstraints(double velo, double accel, double decel) {
        this.velo = Math.abs(velo);
        this.accel = Math.abs(accel);
        this.decel = Math.abs(decel);
    }

    public void convert(double factor) {
        this.velo *= factor;
        this.accel *= factor;
        this.decel *= factor;
    }
}