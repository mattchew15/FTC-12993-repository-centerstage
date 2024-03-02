package org.firstinspires.ftc.teamcode.system.accessory;

public class OuttakeInverseKinematics {
    public double distance = 0;
    public double robotAngle;
    // heightStart/End = height on bd

    // these functions are all intermediate steps
    public double bdAngle() {
        return Math.atan2(Math.sqrt(3) * Math.cos(robotAngle), 1);
    }

    public double h(double heightEnd) {
        return heightEnd * Math.sin(bdAngle());
    }

    public double t(double heightEnd) {
        return heightEnd * Math.cos(bdAngle());
    }
    // these functions are all intermediate steps

    public double pitchEnd(double heightEnd) { // this should be in degrees
        return Math.atan2(h(heightEnd), distance + t(heightEnd));
    }

    public double slideEnd(double heightEnd) {
        return Math.sqrt(Math.pow(h(heightEnd),2) + Math.pow(distance + t(heightEnd), 2));
    }

    public double railEnd(double heightStart, double heightEnd, double railStart) { // rail start needs to be cached. Alternatively use servo.getposition into the parameter
        return (heightEnd - heightStart) * Math.cos(bdAngle()) * Math.sin(robotAngle) + railStart;
    }
}
