package Main.src.treamcode;

import static Main.src.treamcode.RobotMovement.followCurve;

import java.util.ArrayList;

public class MyOpMode extends OpMode{


    @Override
    public void init() {

    }

    @Override
    public void loop() {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0, 0, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(180, 180, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(220, 180, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(280, 50, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(180, 0, 1.0, 1.0, 50, Math.toRadians(50), 1.0));

        followCurve(allPoints, Math.toRadians(90));
    }
}
