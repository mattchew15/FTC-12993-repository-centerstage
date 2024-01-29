package org.firstinspires.ftc.teamcode.system.accessory;

public class AprilTagLocalisation {
    public double cameraOffset = 0;

    public double xPos(double x, double y, double robotAngle) {
        double distance = Math.sqrt(Math.pow(x + cameraOffset, 2) + Math.pow(y, 2));
        double theta = 90 - robotAngle - Math.atan2(y, x);

        return distance * Math.sin(theta);
    }

    public double yPos(double x, double y, double robotAngle) {
        double distance = Math.sqrt(Math.pow(x + cameraOffset, 2) + Math.pow(y, 2));
        double theta = 90 - robotAngle - Math.atan2(y, x);

        return distance * Math.cos(theta);
    }
}
