package org.firstinspires.ftc.teamcode.system.paths.P2P;

import androidx.annotation.NonNull;

public class Vector {
    private double x,y,z;

    public Vector(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector(double x, double y){
        this(x,y,0);
    }

    public Vector(){
        this(0,0,0);
    }

    public Vector(Vector otherVector){
        this.x = otherVector.x;
        this.y = otherVector.y;
        this.z = otherVector.z;
    }

    public static Vector fromAngleAndMagnitude(double t, double m){
        return new Vector(m*Math.cos(t), m*Math.sin(t));
    }

    //t1 is angle in xy plane, t2 is angle with xy plane
    public static Vector fromAngleAndMagnitude(double t1, double t2, double m){
        return new Vector(Math.cos(t1)*Math.cos(t2)*m, Math.sin(t1)*Math.cos(t2)*m, Math.sin(t2)*m);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double getMagnitude(){
        return Math.sqrt(x*x + y*y + z*z);
    }

    public Vector plus(Vector other){
        return new Vector(x + other.getX(), y + other.getY(), z + other.getZ());
    }

    public void scaleToMagnitude(double targetMagnitude){
        double currentMagnitude = getMagnitude();
        scaleBy(1.0/currentMagnitude);
        scaleBy(targetMagnitude);
    }

    public void scaleBy(double a){
        x = x * a;
        y = y * a;
        z = z * a;
    }
    public Vector rotated(double theta)
    {
        return new Vector(Math.cos(theta) * x + Math.sin(theta) * y, Math.cos(theta) * y - Math.sin(theta) * x);
    }

    public Vector scaledBy(double a){
        return new Vector(x * a, y * a, z * a);
    }

    public Vector scaledToMagnitude(double targetMagnitude){
        Vector aux = new Vector(this);
        aux.scaleToMagnitude(targetMagnitude);
        return aux;
    }

    public static Vector  rotateBy(Vector vector, double theta){
        return new Vector(Math.cos(theta) * vector.getX() + Math.sin(theta) * vector.getY(), Math.cos(theta) * vector.getY() - Math.sin(theta) * vector.getX());
    }

    @NonNull
    @Override
    public String toString(){
        return String.valueOf(x) + " " + String.valueOf(y) + " " + String.valueOf(z);
    }
}
