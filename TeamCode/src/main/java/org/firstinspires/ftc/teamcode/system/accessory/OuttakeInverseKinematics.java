package org.firstinspires.ftc.teamcode.system.accessory;

public class OuttakeInverseKinematics {
    public double distance = 0;

    public double bdAngle(double robotAngle) {
        return Math.atan2(Math.sqrt(3) * Math.cos(robotAngle), 1);
    }

    public double h(double heightEnd, double robotAngle) {
        return heightEnd * Math.sin(bdAngle(robotAngle));
    }

    public double t(double heightEnd, double robotAngle) {
        return heightEnd * Math.cos(bdAngle(robotAngle));
    }

    public double slideEnd(double heightEnd, double robotAngle) {
        return Math.atan2(h(heightEnd, robotAngle), distance + t(heightEnd, robotAngle));
    }

    public double pitchEnd(double heightEnd, double robotAngle) {
        return Math.sqrt(Math.pow(h(heightEnd, robotAngle),2) + Math.pow(distance + t(heightEnd, robotAngle), 2));
    }

    public double railEnd(double railStart, double heightEnd, double heightStart) {
        return (railStart * heightEnd)/heightStart;
    }
}
