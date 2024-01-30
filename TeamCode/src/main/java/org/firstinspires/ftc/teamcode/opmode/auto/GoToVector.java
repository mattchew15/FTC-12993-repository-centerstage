package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.PID_X;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.PID_Y;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.powerDebug;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.unwrappedPID_H;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.wrappedPID_H;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MovementVars.updateVars;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot.worldYPosition;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.currentPoint;

import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.goToPosition;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.goToPositionDebug;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics;
import org.firstinspires.ftc.teamcode.system.paths.PurePersuit.Robot;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "RUn to Vector", group = "aTestR")
public class GoToVector extends OpMode
{
    DriveBase drive = new DriveBase();
    List<Integer> lastTrackingEncPositions = new ArrayList<>();
    List<Integer> lastTrackingEncVels = new ArrayList<>();
    StandardTrackingWheelLocalizer localizer;
    public static double TARGET_X = 0, TARGET_Y = 40, TARGET_H = 90;
    public static double START_X = 0, START_Y = 0, START_H = 0;
    public static double ROBOT_X = START_X, ROBOT_Y = START_Y, ROBOT_H = Math.toRadians(START_H);
    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void init()
    {
        TARGET_H = Math.toRadians(TARGET_H);
        ROBOT_H = Math.toRadians(START_H);


        localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions,lastTrackingEncVels);
        drive.initDrivebase(hardwareMap);
        drive.drivebaseSetup();
        localizer.setPoseEstimate(new Pose2d(START_X, START_Y, START_H));
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop()
    {
        localizer.update();
        Pose2d pose = localizer.getPoseEstimate();

        ROBOT_X = pose.getX();
        ROBOT_Y = pose.getY();
        ROBOT_Y = pose.getHeading();

        double[] movements = goToPositionDebug(TARGET_X, TARGET_Y, 0.4, Math.toRadians(90), 0.5);
        //TARGET_H = goToHeading(TARGET_X, TARGET_Y);

        double[] powers = powerDebug(movements[0], movements[1], movements[2]); //DriveTrainKinematics.driveToPosition(TARGET_X, TARGET_Y, TARGET_H, START_X, START_Y , START_H, telemetry);

        drive.FL.setPower(powers[0]);
        drive.FR.setPower(powers[1]);
        drive.BL.setPower(powers[2]);
        drive.BR.setPower(powers[3]);

        telemetry.addData("X", ROBOT_X);
        telemetry.addData("Y", ROBOT_Y);
        telemetry.addData("Theta", ROBOT_H);

        telemetry.addData("following x", TARGET_X);
        telemetry.addData("following y", TARGET_Y);
        telemetry.addData("following H", TARGET_H);


        telemetry.update();


    }
}
