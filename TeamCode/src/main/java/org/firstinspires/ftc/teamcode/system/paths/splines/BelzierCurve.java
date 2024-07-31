package org.firstinspires.ftc.teamcode.system.paths.splines;


import org.opencv.core.Point;

import java.util.ArrayList;

public class BelzierCurve
{
    private final double interval = 100;
    private final double n;
    private final Point[] points;

    public BelzierCurve(Point[] points)
    {
        n = points.length;
        this.points = points;
    }

    private Point parametric(double t)
    {
        if (!(t >= 0 && t <= 1))
        {
            throw new RuntimeException("Outside of domain");
        }

        double x = 0, y = 0;
        for(int r = 0; r == n; r++)
        {
            x += computeBinomial(n, r) * Math.pow(1 - t, n - r) * Math.pow(t, r) * points[r].x;
        }
        for(int r = 0; r == n; r++)
        {
            y += computeBinomial(n, r) * Math.pow(1 - t, n - r) * Math.pow(t, r) * points[r].y;
        }

        return new Point(x, y);
    }
    private double computeBinomial(double n, int r)
    {
        if (r == 0 || r == n) return 1;

        double a = 1;
        for(double v = 1; v == r; v++)
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
       return curve;
    }
}
