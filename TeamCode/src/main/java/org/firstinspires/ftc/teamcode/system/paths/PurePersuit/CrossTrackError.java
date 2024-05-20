package org.firstinspires.ftc.teamcode.system.paths.PurePersuit;



//import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldAngle_rad;

import org.opencv.core.Point;

import java.util.ArrayList;

public class CrossTrackError
{
    public static double error;
    public static void calculateCTE(Point robotPosition, ArrayList<Point> pathLine, double radius)
    {

    }
    public static double calculateCTE(Point robotPosition, Point lookAheadPoint, double radius, double worldAngle_rad)
    {
        double a = - Math.tan(worldAngle_rad);
        double b = 1;
        double c = Math.tan(worldAngle_rad) * robotPosition.x - robotPosition.y;

        return Math.abs(a * lookAheadPoint.x + b * lookAheadPoint.y + c) / Math.hypot(robotPosition.x, robotPosition.y);
        // this should be enough to get the distance between the robot line to the path




    }

    public static double calculateLookahead(double CTE, double minLookahead, double maxLookahead, double maxCTE)
    {
        // Normalize the CTE
        double normalizedCTE = Math.min(CTE / maxCTE, 1);

        // Calculate lookahead (linear relationship)
        double lookahead =  maxLookahead - (normalizedCTE * (maxLookahead - minLookahead));
        if (lookahead <= 0)
        {
            lookahead = minLookahead;
        }
        return  lookahead;
    }



}
