package org.firstinspires.ftc.teamcode.system.paths;


import static java.lang.Math.*;

import org.opencv.core.Point;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class MathFunctions
{
    /**
     * Wraps the able so it is always in the range [-180, 180].
     *
     * @param angle Angle to be wrapped, in radians.
     * @return The wrapped angle, in radians.
     */
    public static double angleWrap(double angle) {
        if (angle > 0)
            return ((angle + Math.PI) % (Math.PI * 2)) - Math.PI;
        else
            return ((angle - Math.PI) % (Math.PI * 2)) + Math.PI;
    }

    /** Generate the waypoints of a given line
     * @param circleCenter Robots position
     * @param radius Look ahead distance
     * @param linePoint1 start point of line
     * @param linePoint2 end point of line*/
    public static ArrayList<Point> lineCircleIntersection(double radius, Point circleCenter,
                                                          Point linePoint1, Point linePoint2)
    {
        // If lines are to small
        if (abs(linePoint1.y - linePoint2.y) < 0.003)
        {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (abs(linePoint1.x - linePoint2.x) < 0.003)
        {
            linePoint1.x = linePoint2.x + 0.003;
        }
        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);

        double quadraticA = 1.0 + pow(m1, 2);

        // shift the circle to simplify calcs
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * pow(m1,2)* x1);
        double quadraticC = ((pow(m1, 2) * pow(x1, 2))) - (2.0 * y1 * m1 * x1) + pow(y1, 2) - pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<>();
        // If sqrt is negative it will throw a NaN?
        try
        {
            double xRoot1 = (-quadraticB + sqrt(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / 2.0 * quadraticA;
            double yRoot1 = m1 * (xRoot1 - x1) - y1;

            // Add the offset
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x; // should replace this probably

            if (xRoot1 > minX && xRoot1 < maxX)
            {
                allPoints.add(new Point(xRoot1, yRoot1));
            }

            // Same but for the other root
            double xRoot2 = (-quadraticB - sqrt(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / 2.0 * quadraticA;
            double yRoot2 = m1 * (xRoot2 - x1) - y1;

            // Add the offset
            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;


            if (xRoot2 > minX && xRoot2 < maxX)
            {
                allPoints.add(new Point(xRoot2, yRoot2));
            }

        }
        catch (Exception e)
        {
            // I know bad practice but who cares...
        }
        return allPoints;
    }

    // not using this functions because the exception would be thrown here and that is bad
    private double[] quadraticSolver(double a, double b, double c)
    {
        double root1 =(-b + sqrt(pow(b, 2) - (4.0 * a * c))) / 2.0 * a;
        double root2 =(-b - sqrt(pow(b, 2) - (4.0 * a * c))) / 2.0 * a;
        return new double[]{root1, root2};
    }
    private float[] quadraticSolver(float a, float b, float c)
    {

        float root1 = (float) ((-b + sqrt(pow(b, 2) - (4.0 * a * c))) / 2.0 * a);
        float root2 =(float) ((-b - sqrt(pow(b, 2) - (4.0 * a * c))) / 2.0 * a);
        return new float[]{root1, root2};
    }
}

