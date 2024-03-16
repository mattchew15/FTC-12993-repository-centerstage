package org.firstinspires.ftc.teamcode.system.accessory;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeInverseKinematics {
    public final double

            offset = 3,
            slideLength = 11.811;
    public double distance = 15;
    public double
            newDistance,
            varT,
            newT;
    // heightStart/End is vertical height of outtake

    // these functions are all intermediate steps
    public double varL(double new_height, double robotAngle) {
        newDistance = distance / Math.cos(robotAngle);
        varT = 1 / Math.sqrt(3) * new_height;
        newT = (varT + distance) / Math.cos(robotAngle) - newDistance;
        return offset + newDistance + newT;
    }

/*
    public double bdAngle() {
        return Math.atan2(Math.sqrt(3) * Math.cos(robotAngle), 1);
    }

    public double c(double heightStart, double heightEnd) { return (heightEnd - heightStart) / (Math.sqrt(3) * Math.cos(robotAngle)); }
    // these functions are all intermediate steps
 */
    // this needs to be in angles
    public double pitchEnd(double new_height, double robotAngle){
        double pitchEnd = Math.toDegrees(Math.atan2(new_height, varL(new_height, robotAngle)));
        if (pitchEnd > 60){
            return 60;
        } else if (pitchEnd < 20){
            return 20;
        } else {
            return pitchEnd;
        }
    }

    public double slideEnd(double new_height, double robotAngle) {
        double slideEnd = Math.hypot(new_height, varL(new_height, robotAngle) - slideLength); // might have to subtract slideLength
        if (slideEnd > 28){
            return 28;
        } else if (slideEnd < 0){
            return 0;
        } else {
            return slideEnd;
        }
    }

    // rail start needs to be cached. Alternatively use servo.getposition into the parameter
    // public double railEnd(double heightStart, double heightEnd, double railStart) {return c(heightStart, heightEnd) * Math.sin(robotAngle) + railStart; }
}
