package org.firstinspires.ftc.teamcode.system.accessory;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class AprilTagLocalisation {
    public double cameraOffset = 6.5;
    public double aprilTagX, aprilTagY, robotAngle;

    // these functions are all intermediate steps
    public double distance() {return Math.hypot(aprilTagY + cameraOffset, aprilTagX);}

    // returns radians
    public double angle() {return Math.toRadians(robotAngle) + Math.atan2(aprilTagX, aprilTagY + cameraOffset); }
    // these functions are all intermediate steps

    public double xPos() {return distance() * Math.cos(angle()); }

    public double yPos() {return distance() * Math.sin(angle()); }

    public Pose2d getCorrectedPose(double robotAngle, double aprilTagX, double aprilTagY)
    {
        this.aprilTagX = aprilTagX;
        this.aprilTagY = aprilTagY;
        this.robotAngle = robotAngle;

        return new Pose2d(xPos(), yPos());

    }
}
