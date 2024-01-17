package Main.src.treamcode;

import Main.src.org.opencv.core.Point;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double pointLength;
    public double slowDownTurnRadians;
    public double slowDownTurnAmount;
    public boolean reverse;

    public CurvePoint (double x, double y, double moveSpeed, double turnSpeed, double followDistance,
                       double slowDownTurnRadians, double slowDownTurnAmount, boolean reverse){
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
        this.reverse = reverse;
    }

    public CurvePoint(CurvePoint thisPoint){
        x = thisPoint.x;
        y = thisPoint.y;
        moveSpeed = thisPoint.moveSpeed;
        turnSpeed = thisPoint.turnSpeed;
        followDistance = thisPoint.followDistance;
        slowDownTurnRadians = thisPoint.slowDownTurnRadians;
        slowDownTurnAmount = thisPoint.slowDownTurnAmount;
        pointLength = thisPoint.pointLength;
        reverse = thisPoint.reverse;
    }

    public Point toPoint(){
        return new Point(x, y);
    }

    public void setPoint(Point point){
        x = point.x;
        y = point.y;
    }
}
