package org.firstinspires.ftc.teamcode.system.paths.PurePersuit;



import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.lineCircleIntersection;
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
    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition, worldYPosition),
                allPoints.get(0).followDistance);

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }
    // TODO: a reverse option where it gets the furthest intersection from the heading? but the vector would then have a high turn amount
    public static CurvePoint getFollowPointPath (ArrayList<CurvePoint> pathPoint, Point robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoint.get(0));

        for (int i = 0; i < pathPoint.size() - 1; i ++){
            CurvePoint startLine = pathPoint.get(i);
            CurvePoint endLine = pathPoint.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(),
                    endLine.toPoint());

            double closestAngle = 1000000000;

            for (Point thisIntersection : intersections){
                double angle = Math.atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition);
                double deltaAngle = Math.abs(AngleWrap(angle - worldAngle_rad));

                if (deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }

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

        if (distanceToTarget < 10){
            movement_turn = 0;
        }
    }

}
