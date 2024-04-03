package org.firstinspires.ftc.teamcode.system.accessory;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AprilTagLocalisation {
    public double cameraOffset = 5.76;
    public Telemetry telemetry;

    public AprilTagLocalisation(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }

    // these functions are all intermediate steps
    public double distance(double aprilTagX, double aprilTagY) {
        double dis = Math.hypot(aprilTagY + cameraOffset, aprilTagX);
        telemetry.addData("Distance", dis );
        return dis;
    }

    // returns radians
    public double angle(double robotAngle, double aprilTagX, double aprilTagY) {
        double ang = Math.toRadians(robotAngle) + Math.atan(aprilTagX / (aprilTagY + cameraOffset));
        telemetry.addData("Angle", ang);
        return ang;
    }
    // these functions are all intermediate steps

    public double xPos(double robotAngle, double aprilTagX, double aprilTagY) {
        return distance(aprilTagX, aprilTagY) * Math.cos(angle(robotAngle, aprilTagX, aprilTagY)); }

    public double yPos(double robotAngle, double aprilTagX, double aprilTagY) {
        return distance(aprilTagX, aprilTagY) * Math.sin(angle(robotAngle, aprilTagX, aprilTagY)); }

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
