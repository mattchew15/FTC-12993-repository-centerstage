package org.firstinspires.ftc.teamcode.system.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import static org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem.RAIL_RANGE;

public class OuttakeInverseKinematics {
    OuttakeSubsystem outtakeSubsystem;
    public final double
            offset = 3,
            slideLength = 11.811;
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

    // these functions are all intermediate steps
    public double varL(double new_height, double robotAngle) {
        newDistance = distance / Math.cos(robotAngle);
        varT = new_height / Math.sqrt(3);
        newT = (varT + distance) / Math.cos(robotAngle) - newDistance;
        return offset + newDistance + newT;
    }

    // this needs to be in angles
    public double pitchEnd(double new_height, double robotAngle) {
        robotAngleRad = Math.toRadians(robotAngle);
        pitchEnd = Math.toDegrees(Math.atan2(new_height, varL(new_height, robotAngleRad)));
        if (pitchEnd > 60) {
            return 60;
        } else if (pitchEnd < 20) {
            return 20;
        } else {
            return pitchEnd;
        }
    }

    public double slideEnd(double new_height, double robotAngle) {
        robotAngleRad = Math.toRadians(robotAngle);
        slideEnd = Math.hypot(new_height, varL(new_height, robotAngleRad) - slideLength); // might have to subtract slideLength
        if (slideEnd > 28) {
            return 28;
        } else if (slideEnd < 0) {
            return 0;
        } else {
            return slideEnd;
        }
    }

    // heightStart and railStart needs to be cached. Alternatively use servo.getposition into the parameter
    public double railEndInch(double heightStart, double heightEnd, double railStart, double robotAngle) {
        robotAngleRad = Math.toRadians(robotAngle);
        varC = (heightEnd - heightStart) / (Math.sqrt(3) * Math.cos(robotAngleRad));
        return varC * Math.sin(robotAngleRad) + railStart;
    }

    /*
    public double railEndTicks(double heightStart, double heightEnd, double railStart, double robotAngle) {
        return outtakeSubsystem.railInchesToTicks(railEndInch(heightStart, heightEnd, railStart, robotAngle));
    }
     */
}
