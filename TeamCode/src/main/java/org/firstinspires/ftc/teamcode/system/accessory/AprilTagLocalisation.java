package org.firstinspires.ftc.teamcode.system.accessory;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class AprilTagLocalisation {
    public double cameraOffset = 6.5;

    // these functions are all intermediate steps
    public double distance(double aprilTagX, double aprilTagY) {return Math.hypot(aprilTagY + cameraOffset, aprilTagX);}

    // returns radians
    public double angle(double robotAngle, double aprilTagX, double aprilTagY) {return Math.toRadians(robotAngle) + Math.atan2(aprilTagX, aprilTagY + cameraOffset); }
    // these functions are all intermediate steps

    public double xPos(double robotAngle, double aprilTagX, double aprilTagY) {return distance(aprilTagX, aprilTagY) * Math.cos(angle(robotAngle, aprilTagX, aprilTagY)); }

    public double yPos(double robotAngle, double aprilTagX, double aprilTagY) {return distance(aprilTagX, aprilTagY) * Math.sin(angle(robotAngle, aprilTagX, aprilTagY)); }

/*
    public Pose2d getCorrectedPose(double robotAngle, double aprilTagX, double aprilTagY)
    {
        this.aprilTagX = aprilTagX;
        this.aprilTagY = aprilTagY;
        this.robotAngle = robotAngle;

        return new Pose2d(xPos(), yPos());

    }
*/
}
