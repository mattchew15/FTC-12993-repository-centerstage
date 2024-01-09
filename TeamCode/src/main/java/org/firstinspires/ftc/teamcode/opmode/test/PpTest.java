package org.firstinspires.ftc.teamcode.opmode.test;

import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.CmToInches;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.inchesToCm;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.followCurve;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldYPosition;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.paths.PurePersuit.CurvePoint;
import org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "PP test", group = "TestR")
public class PpTest extends OpMode
{
    List<Integer> lastTrackingEncPositions = new ArrayList<>(); //
    List<Integer> lastTrackingEncVels = new ArrayList<>();
    StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

    ArrayList<CurvePoint> allPoints;
    DriveBase drive = new DriveBase();
    @Override
    public void init() {

        drive.initDrivebase(hardwareMap);
        localizer.setPoseEstimate(new Pose2d(CmToInches(worldXPosition), CmToInches(worldYPosition), worldAngle_rad));
        allPoints = new ArrayList<>();
        //allPoints.add(new CurvePoint(0, 0, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(150, 150, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        //allPoints.add(new CurvePoint(0, 0, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(20, 150, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(20, 70, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(320, 70, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(280, 150, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        //allPoints.add(new CurvePoint(150, 150, 1.0, 1.0, 50, Math.toRadians(50), 1.0));

    }

    @Override
    public void loop() {
        localizer.update();
        Pose2d pose = localizer.getPoseEstimate();
        worldXPosition = inchesToCm(pose.getX());
        worldYPosition = inchesToCm(pose.getY());
        worldAngle_rad = pose.getHeading();

        followCurve(allPoints, Math.toRadians(90));
        double[] powerPID = DriveTrainKinematics.powerPID();
        double[] powerVec = DriveTrainKinematics.powerFromVector();
        telemetry.addData("PowerVec:"," LeftF " + powerVec[0] + " RightF " + powerVec[1] +" LeftB " + powerVec[2] + " RightF " + powerVec[3]);
        telemetry.addData("PowerPID: ", " LeftF " + powerPID[0] + " RightPID " + powerPID[1] +" LeftB " + powerPID[2] + " RightF " + powerPID[3]);
        drive.FL.setPower(powerPID[0]);
        drive.FR.setPower(powerPID[1]);
        drive.BL.setPower(powerPID[2]);
        drive.BR.setPower(powerPID[3]);

    }

}
