package org.firstinspires.ftc.teamcode.system.accessory;

public class OuttakeInverseKinematics {
    public double distance = 100;
    public double offset = 10;
    public double robotAngle;
    // heightStart/End is vertical height of outtake

    // these functions are all intermediate steps
    public double t(double heightEnd) {return Math.sqrt(3) * heightEnd; }

    public double newDistance() {return distance / Math.cos(robotAngle); }

    public double newT(double heightEnd) {return (t(heightEnd) + distance) / Math.cos(robotAngle) - newDistance(); }

    public double L(double heightEnd) {return offset + newDistance() + newT(heightEnd); }

    public double bdAngle() {
        return Math.atan2(Math.sqrt(3) * Math.cos(robotAngle), 1);
    }

    public double c(double heightStart, double heightEnd) { return (heightEnd - heightStart) / (Math.sqrt(3) * Math.cos(robotAngle)); }
    // these functions are all intermediate steps

    // this needs to be in angles
    public double pitchEnd(double heightEnd) { return Math.atan2(heightEnd, L(heightEnd)); }

    public double slideEnd(double heightEnd) {return Math.sqrt(Math.pow(heightEnd,2) + Math.pow(L(heightEnd), 2)); }

    // rail start needs to be cached. Alternatively use servo.getposition into the parameter
    public double railEnd(double heightStart, double heightEnd, double railStart) {return c(heightStart, heightEnd) * Math.sin(robotAngle) + railStart; }
}
