package org.firstinspires.ftc.teamcode.system.paths.splines;


import org.firstinspires.ftc.teamcode.system.paths.P2P.Vector;
import org.opencv.core.Point;

import java.util.ArrayList;

public class BelzierCurve
{
    private final double interval = 100;
    private final double n;
    private final Point[] points;

    public BelzierCurve(Point[] points)
    {
        n = points.length - 1;
        this.points = points;
    }

    private Point parametric(double t)
    {
        if (!(t >= 0 && t <= 1))
        {
            throw new RuntimeException("Outside of domain");
        }

        double x = 0, y = 0;
        for(int r = 0; r <= n; r++)
        {
            x += computeBinomial(n, r) * Math.pow(1 - t, n - r) * Math.pow(t, r) * points[r].x;
        }
        for(int r = 0; r <= n; r++)
        {
            y += computeBinomial(n, r) * Math.pow(1 - t, n - r) * Math.pow(t, r) * points[r].y;
        }

        return new Point(x, y);
    }
    private double computeBinomial(double n, int r)
    {
        if (r == 0 || r == n) return 1;

        double a = 1;
        for(double v = 1; v <= r; v++)
        {
            a *= (n - v + 1) / v;
        }
        return a;
    }

    public ArrayList<Point> returnCurve()
    {
        ArrayList<Point> curve = new ArrayList<>();
        double t = 0;
        while (t <= 1) {
            curve.add(parametric(t));
            t += 1.0 / interval;
        }
        curve.add(parametric(1));
       return curve;
    }

    private Point parametricDerived(double t)
    {
        if (!(t >= 0 && t <= 1))
        {
            throw new RuntimeException("Outside of domain");
        }

        double x = 0, y = 0;
        for(int r = 0; r <= n; r++)
        {
            //x += computeBinomial(n, r) * -1 * (r-n) * Math.pow(1 - t, n - r - 1) * r * Math.pow(t, r - 1) * points[r].x;
            x += computeBinomial(n, r) * points[r].x * Math.pow(1-t, n - r - 1) * Math.pow(t, r - 1) * (r - t * n);

            //y += computeBinomial(n, r) * -1 * (r-n) * Math.pow(1 - t, n - r - 1) * r * Math.pow(t, r - 1) * points[r].y;
            y += computeBinomial(n, r) * points[r].y * Math.pow(1 - t, n - r - 1) * Math.pow(t, r - 1) * (r - t * n);
        }

        return new Point(x, y);
    }
/*    private Point parametricDerived2(double t)
    {
        if (!(t >= 0 && t <= 1))
        {
            throw new RuntimeException("Outside of domain");
        }

        double x = 0, y = 0;
        for(int r = 0; r <= n; r++)
        {
            //x += computeBinomial(n, r) * -1 * (r-n) * Math.pow(1 - t, n - r - 1) * r * Math.pow(t, r - 1) * points[r].x;
            //x += computeBinomial(n, r) * points[r].x * Math.pow(1-t, n - r - 1) * Math.pow(t, r - 1) * (r - t * n);
            x += computeBinomial(n, r) * (-(n - r) * Math.pow((1 - t), (n - r - 1)) * Math.pow(t, r)
                    + r * Math.pow(t, (r - 1)) * Math.pow((1 - t), (n - r))) * points[r].x;

            y += computeBinomial(n, r) * (-(n - r) * Math.pow((1 - t), (n - r - 1)) * Math.pow(t, r)
                    + r * Math.pow(t, (r - 1)) * Math.pow((1 - t), (n - r))) * points[r].y;
            //y += computeBinomial(n, r) * -1 * (r-n) * Math.pow(1 - t, n - r - 1) * r * Math.pow(t, r - 1) * points[r].y;
            //y += computeBinomial(n, r) * points[r].y * Math.pow(1 - t, n - r - 1) * Math.pow(t, r - 1) * (r - t * n);
        }

        return new Point(x, y);
    }*/
    public ArrayList<Point> returnDerivedCurve()
    {
        ArrayList<Point> curve = new ArrayList<>();
        double t = 0;
        while (t <= 1) {
            if (t != 0) curve.add(parametricDerived(t));
            t += 1.0 / interval;
        }
        return curve;
    }

    public Vector getTangentialVector(double t)
    {
        Point point = parametricDerived(t);
        return new Vector(point.x, point.y);
    }
    public double getTangentialHeading(double t)
    {
        Point point = parametricDerived(t);
        return Math.atan2(point.y, point.x);
    }

    /**Keep in mind result in rad **/
    public ArrayList<Double> returnTangentialHeading()
    {
        ArrayList<Double> curve = new ArrayList<>();
        double t = 0;
        while (t <= 1) {
            if (t != 0) curve.add(getTangentialHeading(t));
            t += 1.0 / interval;
        }
        return curve;
    }
    public ArrayList<Vector> returnTangentialVectors()
    {
        ArrayList<Vector> curve = new ArrayList<>();
        double t = 0;
        while (t <= 1) {
            if (t != 0) curve.add(getTangentialVector(t));
            t += 1.0 / interval;
        }
        return curve;
    }

}
