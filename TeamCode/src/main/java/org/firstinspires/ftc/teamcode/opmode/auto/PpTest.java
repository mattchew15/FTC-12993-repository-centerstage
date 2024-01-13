package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.PID_X;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.PID_Y;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.unwrappedPID_H;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.wrappedPID_H;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.CmToInches;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.inchesToCm;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.movement_y;

import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.currentPoint;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.finished;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.followCurve;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldYPosition;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.goToPosition;


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
        localizer.setPoseEstimate(new Pose2d(worldXPosition, worldYPosition, worldAngle_rad));
        allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(59, 0, 1.0, 0.8, 15, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(59, 39, 1.0, 0.8, 15, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(94, 39, 1.0, 0.8, 27, Math.toRadians(50), 1.0));

    }

    @Override
    public void loop() {
        localizer.update();
        Pose2d pose = localizer.getPoseEstimate();
        worldXPosition = pose.getX();
        worldYPosition = pose.getY();
        worldAngle_rad = pose.getHeading();


        followCurve(allPoints, Math.toRadians(90));
        double[] powerPID = DriveTrainKinematics.powerPID();

        double[] powerNormal = DriveTrainKinematics.normalPID();
        double[] powerUNCLAM = DriveTrainKinematics.UNCLAMPED_PowerPID();
        telemetry.addData("PowerPID: ", " LeftF " + powerPID[0] + " RightF " + powerPID[1] +" LeftB " + powerPID[2] + " Rightb " + powerPID[3]);
        //telemetry.addData("powerUNCLAM: ", " LeftF " + powerUNCLAM[0] + " RightF " + powerUNCLAM[1] +" LeftB " + powerUNCLAM[2] + " Rightb " + powerUNCLAM[3]);
        // maybe this needs to be negative

        drive.FL.setPower(powerPID[0]);
        drive.FR.setPower(powerPID[1]);
        drive.BL.setPower(powerPID[2]);
        drive.BR.setPower(powerPID[3]);




        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Theta", pose.getHeading());

        telemetry.addData("Point following", currentPoint);

        telemetry.addData("Movement X", movement_x);
        telemetry.addData("Movement Y", movement_y);
        telemetry.addData("Turn", movement_turn);

        telemetry.addData("PID X", PID_X);
        telemetry.addData("PID Y", PID_Y);
        telemetry.addData("PID Wrapped", wrappedPID_H);
        telemetry.addData("PID unwrapped", unwrappedPID_H);

        telemetry.update();

    }

}
