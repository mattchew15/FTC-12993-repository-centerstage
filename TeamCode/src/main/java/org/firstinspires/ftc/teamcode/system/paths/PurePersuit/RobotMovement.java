package org.firstinspires.ftc.teamcode.system.paths.PurePersuit;



import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.CrossTrackError.calculateCTE;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.CrossTrackError.calculateLookahead;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.extendVector;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.lineCircleIntersection;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.retractVector;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldYPosition;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.angleToPoint;
import androidx.core.math.MathUtils;

import org.opencv.core.Point;

import java.util.ArrayList;




public class RobotMovement {
    //public static boolean finished = false;
    public static CurvePoint currentPoint;
    public static CurvePoint lastPoint;
    public static boolean finished = false;
    public static double FINAL_HEADING;
    public static double lookAheadDis;

    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        if (lookAheadDis == 0) // the initial one should always be the first
        {
            lookAheadDis = allPoints.get(0).followDistance;
        }
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition, worldYPosition),
                lookAheadDis);


        //lookAheadDis = calculateLookahead(calculateCTE(new Point(worldXPosition, worldYPosition), followMe.toPoint(), lookAheadDis), 10, 25, 5);


        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
   /*
        if (finished)
        {
            CurvePoint startPoint = allPoints.get(allPoints.size() -2);
            CurvePoint endPoint = allPoints.get(allPoints.size() - 1);
            CurvePoint finalPoint = extendVector(startPoint, endPoint);

            goToHeading(finalPoint.x, finalPoint.y, finalPoint.moveSpeed, followAngle, finalPoint.turnSpeed);
        }

         */

        lastPoint = currentPoint;

        currentPoint = followMe;

    }
    // TODO: a reverse option where it gets the furthest intersection from the heading? but the vector would then have a high turn amount
    public static CurvePoint getFollowPointPath (ArrayList<CurvePoint> pathPoint, Point robotLocation, double followRadius){
        CurvePoint followMe;
        //this makes the robot follow the previous point followed

        if (lastPoint != null)
        {
            followMe = lastPoint;
        }
        else // if at the intersect
        {
            followMe = new CurvePoint(pathPoint.get(0));
        }




        for (int i = 0; i < pathPoint.size() - 1; i ++)
        {
            CurvePoint startLine = pathPoint.get(i);
            CurvePoint endLine = pathPoint.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(),
                    endLine.toPoint());


            double closestAngle = 100000000;

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
                    finalStartLine = retractVector(finalStartLine, finalEndLine);

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

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));
        angleToPoint = relativeAngleToPoint; // there was a * distance which is stupid ?? i am stupid

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        movement_turn = MathUtils.clamp(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;


        if (distanceToTarget < 2 ){
            movement_turn = 0;
        }
    }
    // Maybe this was not necessary but i like this implementation
    public static double goToHeading(double x, double y)
    {

/*
        double absoluteAngleToTarget = Math.atan2(y-worldYPosition, x-worldXPosition);

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));
        angleToPoint = relativeAngleToPoint;


        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        movement_turn = MathUtils.clamp(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

    }



 */
        if (finished)
        {
            return FINAL_HEADING;
        }
        else
        {
             return Math.atan2(y-worldYPosition, x-worldXPosition);

           // return AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        }

    }

}
