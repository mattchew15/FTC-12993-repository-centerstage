package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.CmToInches;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.inchesToCm;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.currentPoint;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.finished;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.followCurve;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldYPosition;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.paths.PurePersuit.CurvePoint;
import org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics;
import org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "PP test", group = "TestR")
public class PpTest extends OpMode
{
    List<Integer> lastTrackingEncPositions = new ArrayList<>(); //
    List<Integer> lastTrackingEncVels = new ArrayList<>();
    StandardTrackingWheelLocalizer localizer;

    ArrayList<CurvePoint> allPoints;
    DriveBase drive = new DriveBase();
    @Override
    public void init() {
        localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        drive.initDrivebase(hardwareMap);
        drive.drivebaseSetup();
        Robot robot = new Robot(); // init the pose
        finished = false;
        localizer.setPoseEstimate(new Pose2d(CmToInches(worldXPosition), CmToInches(worldYPosition), worldAngle_rad));
        allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(150, 0, 1.0, 0.8, 70, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(150, 100, 1.0, 0.8, 70, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(-240, 100, 1.0, 0.8, 70, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(-340, 100, 1.0, 0.8, 10, Math.toRadians(50), 1.0));

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
        double[] powerNormal = DriveTrainKinematics.normalPID();
        telemetry.addData("PowerPID: ", " LeftF " + powerPID[0] + " RightF " + powerPID[1] +" LeftB " + powerPID[2] + " Rightb " + powerPID[3]);
        telemetry.addData("PowerNormal: ", " LeftF " + powerNormal[0] + " RightF " + powerNormal[1] +" LeftB " + powerNormal[2] + " Rightb " + powerNormal[3]);
        drive.FL.setPower(-powerPID[0]);
        drive.FR.setPower(-powerPID[1]);
        drive.BL.setPower(-powerPID[2]);
        drive.BR.setPower(-powerPID[3]);
        telemetry.addData("X", inchesToCm(pose.getX()));
        telemetry.addData("Y", inchesToCm(pose.getY()));
        telemetry.addData("Theta", inchesToCm(pose.getHeading()));
        telemetry.addData("Point following", currentPoint);
        telemetry.update();

    }

}
