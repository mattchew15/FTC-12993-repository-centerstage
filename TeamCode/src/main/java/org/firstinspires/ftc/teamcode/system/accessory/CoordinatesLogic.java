package org.firstinspires.ftc.teamcode.system.accessory;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// coordinate stuff
public class CoordinatesLogic {


    public double pointToTarget(Pose2d target, Pose2d current) {
        Vector2d difference = target.vec().minus(current.vec());
        // Obtain the target angle for feedback and derivative for feedforward
        double theta = difference.angle();//tan-1
        double thetaError = theta - current.getHeading();

        return Angle.normDelta(thetaError);
    }

    public double pointToTargetTurret(Pose2d target, Pose2d current, double turretRotations, Telemetry telemetry) {
        Vector2d difference = target.vec().minus(current.vec());
        // Obtain the target angle for feedback and derivative for feedforward
        double theta = difference.angle();//tan-1
        // telemetry.addData("coordinate angle (tan-1)", theta);
        double thetaError = theta - ((turretRotations+current.getHeading()) + Math.toRadians(180)); // turretRotations should be in degrees
        // telemetry.addData("current heading", current.getHeading());
        // telemetry.addData("turretrotations + heading + 180", (turretRotations+current.getHeading()) + Math.toRadians(180));
        return Angle.normDelta(thetaError);
    }

    public Pose2d getTurretPosition(double turretXOffset, double turretYOffset, Pose2d currentPose, Telemetry telemetry){
        double r = Math.sqrt(Math.pow(turretXOffset,2)+Math.pow(turretYOffset,2));
        double offsetTheta = currentPose.getHeading() + (Math.toRadians(180) - Math.atan2(turretYOffset,turretXOffset));
        Pose2d turretPosition = new Pose2d(currentPose.getX() + r*Math.cos(offsetTheta),currentPose.getY() + r*Math.sin(offsetTheta),currentPose.getHeading());
        // telemetry.addData("turretPosition x", turretPosition.getX());
        // telemetry.addData("turretPosition y", turretPosition.getY());
        return turretPosition;
    }

}
