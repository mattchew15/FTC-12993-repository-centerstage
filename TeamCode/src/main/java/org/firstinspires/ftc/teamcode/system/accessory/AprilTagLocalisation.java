package org.firstinspires.ftc.teamcode.system.accessory;

public class AprilTagLocalisation {
    public double cameraOffset = 0;
    public double aprilTagX, aprilTagY, robotAngle;

    // these functions are all intermediate steps
    public double distance() {return Math.sqrt(Math.pow(aprilTagX + cameraOffset, 2) + Math.pow(aprilTagY, 2));}

    public double angle() {return robotAngle + Math.atan2(aprilTagY, aprilTagX + cameraOffset); }
    // these functions are all intermediate steps

    public double xPos() {return distance() * Math.cos(angle()); }

    public double yPos() {return distance() * Math.sin(angle()); }
}
