package Main.src.treamcode;

import static Main.src.com.company.Robot.worldAngle_rad;

import java.util.ArrayList;

import Main.src.org.opencv.core.Point;

public class CrossTrackError
{
    public static double CTE;

    public static double calculateCTE(Point robotPosition, Point lookAheadPoint, double radius)
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

        // Calculate lookahead with a linear relationship
        double lookahead = maxLookahead - (normalizedCTE * (maxLookahead - minLookahead));
        if (lookahead <= 0) // this should never happen but like, be sure
        {
            lookahead = minLookahead;
        }
        return  lookahead;
    }



}
