package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.PID_X;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics.PID_Y;
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
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.RobotMovement.goToHeading;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.paths.PurePersuit.DriveTrainKinematics;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "RUn to position", group = "aTestR")
public class GoToPoint extends OpMode
{
    DriveBase drive = new DriveBase();
    List<Integer> lastTrackingEncPositions = new ArrayList<>();
    List<Integer> lastTrackingEncVels = new ArrayList<>();
    StandardTrackingWheelLocalizer localizer;
    public static double TARGET_X = 0, TARGET_Y = 40, TARGET_H = 90;
    public static double START_X = 0, START_Y = 0, START_H = 90;
    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void init()
    {
        TARGET_H = Math.toRadians(TARGET_H);
        START_H = Math.toRadians(START_H);

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

        START_X = pose.getX();
        START_Y = pose.getY();
        START_H = pose.getHeading();


        //TARGET_H = goToHeading(TARGET_X, TARGET_Y);

        double[] powers = DriveTrainKinematics.driveToPosition(TARGET_X, TARGET_Y, TARGET_H, START_X, START_Y , START_H, telemetry);

        drive.FL.setPower(powers[0]);
        drive.FR.setPower(powers[1]);
        drive.BL.setPower(powers[2]);
        drive.BR.setPower(powers[3]);

        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Theta", pose.getHeading());

        telemetry.addData("following x", TARGET_X);
        telemetry.addData("following y", TARGET_Y);
        telemetry.addData("following H", TARGET_H);


        telemetry.update();


    }
}
