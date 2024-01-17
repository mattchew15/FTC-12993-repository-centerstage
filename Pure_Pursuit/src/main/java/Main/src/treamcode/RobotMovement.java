package Main.src.treamcode;

import static Main.src.com.company.Robot.*;
import static Main.src.treamcode.MathFunctions.*;
import static Main.src.RobotUtilities.MovementVars.*;
import static Main.src.treamcode.DriveTrainKinematics.*;

import java.util.ArrayList;

import Main.src.com.company.ComputerDebugging;
import Main.src.com.company.FloatPoint;
import Main.src.com.company.Range;
import Main.src.org.opencv.core.Point;


public class RobotMovement {
    public static boolean finished = false;
    public static CurvePoint endPoint;
    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        for (int i = 0; i < allPoints.size() - 1; i++){
            ComputerDebugging.sendLine(new FloatPoint(allPoints.get(i).x, allPoints.get(i).y),
                    new FloatPoint(allPoints.get(i+1).x, allPoints.get(i+1).y));
        }
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition, worldYPosition),
                allPoints.get(0).followDistance);

        ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x, followMe.y));

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
        if (finished)
        {
            CurvePoint startPoint = allPoints.get(allPoints.size() -2);
            CurvePoint endPoint = allPoints.get(allPoints.size() - 1);
            CurvePoint finalPoint = extendVector(startPoint, endPoint);

            goToHeading(finalPoint.x, finalPoint.y, finalPoint.moveSpeed, followAngle, finalPoint.turnSpeed);
        }

    }

    // TODO: a reverse option where it gets the furthest intersection from the heading? but the vector would then have a high turn amount
    public static CurvePoint getFollowPointPath (ArrayList<CurvePoint> pathPoint, Point robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoint.get(0));

        for (int i = 0; i < pathPoint.size() - 1; i ++)
        {
            CurvePoint startLine = pathPoint.get(i);
            CurvePoint endLine = pathPoint.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(),
                    endLine.toPoint());


            double closestAngle = 1000000000;

            for (Point thisIntersection : intersections)
            {
                double angle = Math.atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition);
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - worldAngle_rad));

                if (deltaAngle < closestAngle)
                {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);

                    CurvePoint finalStartLine = pathPoint.get(pathPoint.size() - 2);
                    CurvePoint finalEndLine = pathPoint.get(pathPoint.size() - 1);
                    finalStartLine = VectorVectorVector(finalStartLine, finalEndLine);
                    ComputerDebugging.sendKeyPoint(new FloatPoint(finalStartLine.x, finalStartLine.y));
                    ComputerDebugging.sendLine(new FloatPoint(finalStartLine.x, finalStartLine.y), new FloatPoint(finalEndLine.x, finalEndLine.y));

                            //Lazy stuff...
                    ArrayList<Point> interFinal = lineCircleIntersection(thisIntersection,0.2, finalStartLine.toPoint(), finalEndLine.toPoint());
                    if (interFinal.size() > 0)
                    {
                        finished = true;
                    }
                }
            }

        }
        if (finished)
        {
            followMe = new CurvePoint(pathPoint.get(pathPoint.size()-1));

        }
        return followMe;
    }



    /**
     *
     * @param x
     * @param y
     * @param movementSpeed
     */
    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {

        double distanceToTarget = Math.hypot(x-worldXPosition, y-worldYPosition);

        double absoluteAngleToTarget = Math.atan2(y-worldYPosition, x-worldXPosition);

        // (90 degrees is forwards and the preferred angle
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));
        angleToPoint = relativeAngleToPoint; // removed distance in here

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

        if (distanceToTarget < 10 ){
            movement_turn = 0;
        }
    }
    // Maybe this was not necessary but i like this implementation
    public static void goToHeading(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {


        double absoluteAngleToTarget = Math.atan2(y-worldYPosition, x-worldXPosition);

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));
        angleToPoint = relativeAngleToPoint;


        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

    }

}
