package Main.src.treamcode;


import static java.lang.Math.max;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import java.util.ArrayList;
import Main.src.org.opencv.core.Point;

public class MathFunctions {
    /**
     * makes sure an angle is within the range -18to 180 degrees
     * @param angle
     * @return
     **/
    public static double AngleWrap(double angle) {
        while (angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        return angle;
    }
    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius,
                                                          Point linePoint1, Point linePoint2){
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003){
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003){
            linePoint1.x = linePoint2.x + 0.003;
        }

        double m1 = (linePoint2.y - linePoint1.y)/(linePoint2.x - linePoint1.x);

        double quadraticA = 1.0 + Math.pow(m1,2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1,2) * x1);

        double quadraticC = (Math.pow(m1,2) * Math.pow(x1,2)) - (2.0*y1*m1*x1) + Math.pow(y1,2) - Math.pow(radius,2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try{
            double xRoot1 = (-quadraticB + sqrt(Math.pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            //put back offset
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;

            if (xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Point(xRoot1,yRoot1));
            }

            double xRoot2 = (-quadraticB - sqrt(Math.pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if (xRoot2 > minX && xRoot2 < maxX){
                allPoints.add(new Point(xRoot2,yRoot2));
            }

        }catch (Exception e){

        }
        return allPoints;
    }
    // this works fine? but could be better with a scale being the follow radius + a small value
    public static CurvePoint extendVector(CurvePoint start, CurvePoint end)
    {
        Point d = new Point(end.x - start.x, end.y - start.y);
        double u = sqrt(pow(d.x, 2) + pow(d.y, 2));
        Point n = new Point(d.x / u, d.y / u);
        double scalar = end.followDistance;
        n = new Point(n.x * scalar, n.y * scalar);
        n = new Point(end.x + n.x, end.y + n.y);
        CurvePoint f = new CurvePoint(end);
        f.setPoint(n);
        return f;
    }
    public static CurvePoint retractVector(CurvePoint start, CurvePoint end)
    {
        Point d = new Point(end.x - start.x, end.y - start.y);
        double u = sqrt(pow(d.x, 2) + pow(d.y, 2));
        Point n = new Point(d.x / u, d.y / u);
        double scalar = start.followDistance / 5;
        n = new Point(n.x * scalar, n.y * scalar);
        n = new Point(start.x + n.x, start.y + n.y);
        CurvePoint f = new CurvePoint(start);
        f.setPoint(n);
        return f;
    }
}
