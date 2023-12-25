package org.firstinspires.ftc.teamcode.system.paths.PurePersuit;


import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;
import static org.firstinspires.ftc.teamcode.system.paths.MathFunctions.lineCircleIntersection;
import static org.firstinspires.ftc.teamcode.system.paths.ROBOT.movementTurn;
import static org.firstinspires.ftc.teamcode.system.paths.ROBOT.movementX;
import static org.firstinspires.ftc.teamcode.system.paths.ROBOT.movementY;
import static org.firstinspires.ftc.teamcode.system.paths.ROBOT.worldThetaPosition;
import static org.firstinspires.ftc.teamcode.system.paths.ROBOT.worldXPosition;
import static org.firstinspires.ftc.teamcode.system.paths.ROBOT.worldYPosition;
import static java.lang.Math.*;

import android.util.Range;

import androidx.core.math.MathUtils;

import org.firstinspires.ftc.teamcode.system.paths.CurvePoint;
import org.opencv.core.Point;

import java.util.ArrayList;


public class RobotMovement
{
    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle)
    {
        // this is for the debugging
        for (int i = 0; i < allPoints.size() - 1; i++)
        {
            // dont know,
        }
        /*
        // I took it out for tests
        // TODO: put this into a function
        double followRadius = allPoints.get(0).followDistance + 0.003; // maybe smaller? this addition it is just to be safe
        CurvePoint startPoint = allPoints.get(-2);
        CurvePoint endPoint = allPoints.get(-1);
        Point v = new Point(endPoint.x - startPoint.x, endPoint.y - startPoint.y);
        double z = sqrt(pow(v.x,2) + pow(v.y,2));
        Point u = new Point(v.x / z, v.y / z);
        Point e = new Point(u.x * followRadius, u.y * followRadius);
        CurvePoint newEndPoint = new CurvePoint(endPoint);
        newEndPoint.setPoint(new Point(e.x + endPoint.x, e.y + endPoint.y));
        allPoints.add(newEndPoint);
        //This extends the last line figure how to not follow just look at the new point
         */

        // this uses the same follow distance for every point,
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition, worldYPosition), allPoints.get(0).followDistance);
        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }

    /** Finds the point in the path to follow*/
    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius)
    {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size() - 1; i ++)
        {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = lineCircleIntersection(followRadius, robotLocation, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 100000000; // Really high number

            for (Point thisIntersection : intersections)
            {
                double angle = atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition);
                double deltaAngle = abs(angleWrap(angle - worldThetaPosition));

                if (deltaAngle < closestAngle)
                {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }

        return followMe;
    }
    // Simply gives how much to turn, and move in x,y
    public static void goToPosition(double x, double y, double movementSpeed, double turnAngle, double preferredAngle)
    {
        double distanceToTarget = hypot(x - worldXPosition, y - worldYPosition);

        double absoluteAngleToTarget = atan2(y - worldYPosition, x - worldXPosition); // Basic trig get the angle from pose to target

        double relativeAngleToPoint = angleWrap(absoluteAngleToTarget - (worldThetaPosition - toRadians(90)));

        double relativeXToPoint = cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = sin(relativeAngleToPoint) * distanceToTarget;

        double percent = abs(relativeXToPoint) + abs(relativeYToPoint);
        double xPower = relativeXToPoint / percent;
        double yPower = relativeYToPoint / percent;

        movementX = xPower * movementSpeed;
        movementY = yPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - toRadians(180) + preferredAngle;

        movementTurn = MathUtils.clamp(relativeTurnAngle /toRadians(30), -1, 1);

        if (distanceToTarget < 7) // this has not been defined being any measurement system, odo will define it
        {
            movementTurn  = 0;
        }

    }

}
