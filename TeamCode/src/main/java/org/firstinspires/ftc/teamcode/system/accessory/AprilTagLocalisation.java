package org.firstinspires.ftc.teamcode.system.accessory;

public class AprilTagLocalisation {
    public double cameraOffset = 0;

    public double distance(double x, double y) {
        return Math.sqrt(Math.pow(x + cameraOffset, 2) + Math.pow(y, 2));
    }

    public double beta(double x, double y, double robotAngle) {
        double alpha = Math.atan2(y, x + cameraOffset);

        return robotAngle + alpha;
    }

    public double xPos(double x, double y, double robotAngle) {
        return distance(x, y) * Math.cos(beta(x, y, robotAngle));
    }

    public double yPos(double x, double y, double robotAngle) {
        return distance(x, y) * Math.sin(beta(x, y, robotAngle));
    }
}
