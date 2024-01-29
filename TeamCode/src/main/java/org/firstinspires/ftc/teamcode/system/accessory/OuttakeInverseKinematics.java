package org.firstinspires.ftc.teamcode.system.accessory;

public class OuttakeInverseKinematics {
    public double getSlideEnd(double pitchStart, double pitchEnd, double slideStart, double robotAngle) {
        double bdAngle = Math.atan2(Math.sqrt(3) * Math.cos(robotAngle), 1);

        return slideStart * (Math.sin(180 + pitchStart - bdAngle) / Math.sin(bdAngle - pitchEnd));
    }

    public double getRailChange(double pitchStart, double pitchEnd, double slideStart, double robotAngle) {
        double slideEnd = getSlideEnd(pitchStart, pitchEnd, slideStart, robotAngle);

        return robotAngle * Math.sqrt(Math.pow(slideStart, 2) + Math.pow(slideEnd, 2) -
                2 * slideStart * slideEnd * Math.cos(pitchEnd - pitchStart));
    }
}
