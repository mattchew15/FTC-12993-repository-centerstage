package org.firstinspires.ftc.teamcode.system.paths.P2P;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Pose
{
    private final double x, y, heading;
    private final double tolerance;
    public static double defaultTolerance = 0.5; // half an inch that is rr default

    public Pose(double x, double y, double heading, double tolerance)
    {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.tolerance = tolerance;
    }

    public Pose(double x, double y, double heading)
    {
        this(x, y, heading, defaultTolerance);
    }

    public Pose(double x, double y)
    {
        this(x, y, 0, defaultTolerance);
    }

    public Pose()
    {
        this(0, 0, 0, defaultTolerance);
    }

    public Pose(Pose2d pose2d)
    {
        this(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

    public double getX(){
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public double getTolerance(){
        return tolerance;
    }

    public double getDistance(Pose other){
        return Math.sqrt((other.getX() - x)*(other.getX() - x) + (other.getY() - y)*(other.getY() - y));
    }

    public boolean isReached(Pose other){
        return getDistance(other) <= tolerance;
    }

    public Pose plus(Pose other){
        return new Pose(x + other.x, y + other.y, heading + other.heading);
    }

    public Pose2d toPose2d(){
        return new Pose2d(x,y,heading);
    }
    @NonNull
    @Override
    public String toString(){
        return String.valueOf(x) + " " + String.valueOf(y) + " " + String.valueOf(heading);
    }
}