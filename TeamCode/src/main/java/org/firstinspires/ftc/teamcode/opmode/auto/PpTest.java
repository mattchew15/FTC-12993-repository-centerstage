package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.PID_X;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.PID_Y;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.unwrappedPID_H;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.wrappedPID_H;


import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.movement_y;

import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.FINAL_HEADING;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.currentPoint;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.finished;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.followCurve;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldYPosition;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.goToHeading;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.goToPosition;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.paths.PurePersuit.CurvePoint;
import org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics;
import org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot;

import java.util.ArrayList;
import java.util.List;
@Config
@Autonomous(name = "PP test", group = "aTestR")
public class PpTest extends OpMode
{
    List<Integer> lastTrackingEncPositions = new ArrayList<>(); //
    List<Integer> lastTrackingEncVels = new ArrayList<>();
    StandardTrackingWheelLocalizer localizer;
    TelemetryPacket packet;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    LoopTime loopTime = new LoopTime();


    ArrayList<CurvePoint> allPoints;
    DriveBase drive = new DriveBase();
    @Override
    public void init() {
        localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        packet = new TelemetryPacket();
        drive.initDrivebase(hardwareMap);
        drive.drivebaseSetup();
        Robot robot = new Robot(); // init the pose
        finished = false;


        localizer.setPoseEstimate(new Pose2d(worldXPosition, worldYPosition, worldAngle_rad));
        allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(59, 0, 0.5, 1, 15, Math.toRadians(50), 1.0));
        //allPoints.add(new CurvePoint(90, 0, 1.0, 1, 3, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(59, 40, 0.5, 1, 15, Math.toRadians(50), 1.0));
        allPoints.add(new CurvePoint(85, 40, 0.5, 1, 15, Math.toRadians(50), 1.0));
        //allPoints.add(new CurvePoint(80, 37, 0.5, 1, 10, Math.toRadians(50), 1.0));
        for (int i = 0; i < allPoints.size() - 1; i++)
        {
            packet.fieldOverlay().setFill("red").strokeLine(allPoints.get(i).x - 50, allPoints.get(i).y - 25, allPoints.get(i+1).x - 50, allPoints.get(i+1).y - 25);

        }
        dashboard.sendTelemetryPacket(packet);
        FINAL_HEADING = Math.toRadians(0);

    }

    @Override
    public void loop() {
        packet = new TelemetryPacket();
        localizer.update();

        Pose2d pose = localizer.getPoseEstimate();

        worldXPosition = pose.getX();
        worldYPosition = pose.getY();
        worldAngle_rad = pose.getHeading();

        followCurve(allPoints, Math.toRadians(90));

        for (int i = 0; i < allPoints.size() - 1; i++)
        {
            packet.fieldOverlay().setFill("red").strokeLine(allPoints.get(i).x - 50, allPoints.get(i).y - 25, allPoints.get(i+1).x - 50, allPoints.get(i+1).y - 25);

        }
        packet.fieldOverlay().setFill("red").fillRect((worldXPosition - 50) - 5,(worldYPosition - 25) - 5,10, 10);
        packet.fieldOverlay().setStroke("blue").strokeCircle(currentPoint.x - 50,currentPoint.y - 25, 2);
        //packet.fieldOverlay().setFill("green").fillRect((worldXPosition - 50) - 5, worldYPosition - 7, 2, 3);
        double[] power;

        if (!finished)
        {
            // This should drive with gluten free vector util it gets to the last point
             power = DriveTrainKinematics.power();
        }
        else
        {
            power = DriveTrainKinematics.driveToPosition(currentPoint.x, currentPoint.y, FINAL_HEADING,
                    pose.getX(), pose.getY(), pose.getHeading(),
                    telemetry);
        }
        telemetry.addData("PowerPID: ", " LeftF " + power[0] + " RightF " + power[1] +" LeftB " + power[2] + " Rightb " + power[3]);


        drive.FL.setPower(power[0]);
        drive.FR.setPower(power[1]);
        drive.BL.setPower(power[2]);
        drive.BR.setPower(power[3]);




        loopTime.updateLoopTime(telemetry);
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Theta", pose.getHeading());

        telemetry.addData("Point following", currentPoint);
        telemetry.addData("Final theta", FINAL_HEADING);

        telemetry.addData("Movement X", movement_x);
        telemetry.addData("Movement Y", movement_y);
        telemetry.addData("Turn", movement_turn);

        telemetry.addData("PID X", PID_X);
        telemetry.addData("PID Y", PID_Y);
        telemetry.addData("PID Wrapped", wrappedPID_H);
        telemetry.addData("PID unwrapped", unwrappedPID_H);

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);

    }

}
