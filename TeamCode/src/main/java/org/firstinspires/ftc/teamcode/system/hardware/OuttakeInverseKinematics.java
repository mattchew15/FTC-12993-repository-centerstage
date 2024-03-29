package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OuttakeInverseKinematics {
    public static double offset = 3;
    public final double slideLength = 11.811;
    public double
            distance,
            newDistance,
            varT,
            newT,
            robotAngleRad,
            pitchEnd,
            slideEnd,
            varC;
    // heightStart/End is vertical height of outtake

    // this functions is an intermediate step
    public double varL(double heightEnd, double robotAngle) {
        newDistance = distance / Math.cos(robotAngle);
        varT = heightEnd / Math.sqrt(3);
        newT = (varT + distance) / Math.cos(robotAngle) - newDistance;
        return offset + newDistance + newT;
    }

    // this needs to be in angles
    public double pitchEnd(double heightEnd, double robotAngle) {
        robotAngleRad = Math.toRadians(robotAngle);
        pitchEnd = Math.toDegrees(Math.atan2(heightEnd, varL(heightEnd, robotAngleRad)));
        if (pitchEnd > 60) {
            return 60;
        } else if (pitchEnd < 20) {
            return 20;
        } else {
            return pitchEnd;
        }
    }

    public double slideEnd(double heightEnd, double robotAngle) {
        robotAngleRad = Math.toRadians(robotAngle);
        slideEnd = Math.hypot(heightEnd, varL(heightEnd, robotAngleRad) - slideLength); // might have to subtract slideLength
        if (slideEnd > 28) {
            return 28;
        } else if (slideEnd < 0) {
            return 0;
        } else {
            return slideEnd;
        }
    }

    // rail start needs to be cached. Alternatively use servo.getposition into the parameter
    public double railEnd(double heightStart, double heightEnd, double railStart, double robotAngle) {
        robotAngleRad = Math.toRadians(robotAngle);
        varC = (heightEnd - heightStart) / (Math.sqrt(3) * Math.cos(robotAngleRad));
        return varC * Math.sin(robotAngleRad) + railStart;
    }
}
