package Main.src.treamcode;

import static Main.src.treamcode.RobotMovement.finished;
import static Main.src.treamcode.RobotMovement.followCurve;

import java.util.ArrayList;

public class MyOpMode extends OpMode{

    ArrayList<CurvePoint> allPoints;
    @Override
    public void init() {
        allPoints = new ArrayList<>();
        //allPoints.add(new CurvePoint(0, 0, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(150, 150, 1.0, 1.0, 50, Math.toRadians(50), 1.0, false));
        //allPoints.add(new CurvePoint(0, 0, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(20, 150, 1.0, 1.0, 50, Math.toRadians(50), 1.0, false));
        allPoints.add(new CurvePoint(20, 70, 1.0, 1.0, 50, Math.toRadians(50), 1.0, false));
        allPoints.add(new CurvePoint(320, 70, 1.0, 1.0, 50, Math.toRadians(50), 1.0, false));
        allPoints.add(new CurvePoint(280, 150, 1.0, 1.0, 50, Math.toRadians(50), 1.0, false));
        //allPoints.add(new CurvePoint(150, 150, 1.0, 1.0, 50, Math.toRadians(50), 1.0));



    }

    @Override
    public void loop() {
        followCurve(allPoints, Math.toRadians(90));
        //double[] powerPID = DriveTrainKinematics.powerPID();
        double[] powerVec = DriveTrainKinematics.powerFromVector();
        System.out.println("PowerVec: LeftF " + powerVec[0] + " RightF " + powerVec[1] +" LeftB " + powerVec[2] + " RightF " + powerVec[3]);
        //System.out.println("RightF" + DriveTrainKinematics.powerFromVector()[1]);
       // System.out.println("LeftB" + DriveTrainKinematics.powerFromVector()[2]);
       // System.out.println("RightF" + DriveTrainKinematics.powerFromVector()[3]);
        //System.out.println("PowerPID:LeftF" + powerPID[0] + "PowerPID: RightPID" + powerPID[1] +"PowerPID: LeftB" + powerPID[2] + "PowerPID: RightF" + powerPID[3]);
        finished = true;

    }
}
