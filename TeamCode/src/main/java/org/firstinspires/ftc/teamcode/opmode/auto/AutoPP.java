package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Localizer;
import org.firstinspires.ftc.teamcode.system.paths.P2P.MecanumDrive;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;
import org.firstinspires.ftc.teamcode.system.paths.PurePersuit.CurvePoint;

import java.util.ArrayList;

@Autonomous(name = "PP test")
public class AutoPP extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        DriveBase driveBase = new DriveBase(telemetry);
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        TimedSupplier<Double> voltageSupplier = new TimedSupplier<>( voltageSensor::getVoltage, 100);

        MecanumDrive drive = new MecanumDrive(
                driveBase.FL, driveBase.FR, driveBase.BL, driveBase.BR,
                MecanumDrive.RunMode.PP, voltageSupplier);
        drive.setLocalizer(new Localizer(hardwareMap, new Pose(0, 0, Math.toRadians(0)), this));

        //TwoTrackingWheelLocalizer twoTrackingWheelLocalizer = new TwoTrackingWheelLocalizer(hardwareMap)
        ArrayList<CurvePoint> path = new ArrayList<>();
        path.add(new CurvePoint(0, 0, 0.5, 0.5, 5, Math.toRadians(30), Math.toRadians(30)));
        path.add(new CurvePoint(0, -10, 0.5, 0.5, 5, Math.toRadians(30), Math.toRadians(30)));
        drive.setTargetPath(path);
        //drive.setTargetPose(new Pose(10, 0, Math.toRadians(0)));
        waitForStart();
        while(!isStopRequested() && opModeIsActive())
        {
            packet = new TelemetryPacket();
/*
            telemetry.addLine();
            telemetry.addData("Target Heading", drive.getTargetPose().getHeading());
            telemetry.addData("Heading", drive.getLocalizer().getHeading());*/
            telemetry.addLine();
            telemetry.addData("TargetPose", drive.getTargetPose());
            telemetry.addData("PoseEstimate", drive.getLocalizer().getPoseEstimate());
            telemetry.addData("Predicted PoseEstimate", drive.getLocalizer().getPredictedPoseEstimate());
            //telemetry.addData("Is reached", drive.reachedTarget(0.5));
            telemetry.addData("DriveTrain Velocity", drive.getLocalizer().driveTrainVelocity);

            telemetry.addLine();
            telemetry.addData("RunMode", drive.getRunMode());
            telemetry.addData("PowerVector", drive.powerVector);
            telemetry.addData("PowerVector X", drive.powerVector.getX());
            telemetry.addData("PowerVector Y", drive.powerVector.getY());
            telemetry.addData("PowerVector Z", drive.powerVector.getZ());
            telemetry.addData("Gliding vector", drive.getLocalizer().getGlideDelta());

  /*          telemetry.addLine();
            telemetry.addData("Voltage", drive.getVoltage());
            telemetry.addData("FL", drive.FLPower);
            telemetry.addData("FR", drive.FRPower);
            telemetry.addData("BL", drive.BLPower);
            telemetry.addData("BR", drive.BRPower);
*/

/*            packet.fieldOverlay().setFill("red").strokeRect(drive.getLocalizer().getPoseEstimate().getX(),
                    drive.getLocalizer().getPoseEstimate().getY(), 8, 8);
            packet.fieldOverlay().setFill("red").fillCircle(drive.getTargetPose().getX(), drive.getTargetPose().getY(), 2);*/
            //DashboardUtils.drawRobot(canvas, drive.getLocalizer().getPoseEstimate().toPose2d());
            //DashboardUtils.drawTargetPose(canvas, drive.getTargetPose());


            //dashboard.sendTelemetryPacket(packet);
            telemetry.update();


            //if (drive.reachedTarget(0.5)) drive.setTargetPose(drive.getLocalizer().getPoseEstimate());
            drive.update();
        }
    }
}
