package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Localizer;
import org.firstinspires.ftc.teamcode.system.paths.P2P.MecanumDrive;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Vector;


@TeleOp(group="zz")
public class KsTuner extends LinearOpMode {

    //Hardware hardware;

    MecanumDrive drive;

    FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        //hardware = new Hardware(hardwareMap, Hardware.Color.Blue);

        DriveBase driveBase = new DriveBase(telemetry);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        intakeSubsystem.initIntake(hardwareMap);
        intakeSubsystem.intakeHardwareSetup();
        intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();
        driveBase.setUpFloat();

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        TimedSupplier<Double> voltageSupplier = new TimedSupplier<>(voltageSensor::getVoltage, 100);

        drive = new MecanumDrive(
                driveBase.FL, driveBase.FR, driveBase.BL, driveBase.BR,
                MecanumDrive.RunMode.Vector, voltageSupplier);
        drive.setLocalizer(new Localizer(hardwareMap, new Pose(0, 0, Math.toRadians(90)), this));


        //hardware.startThreads(this);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            //hardware.update();

//            hardware.localizer.update();

            drive.setTargetVector(new Vector(0.0001, 0, 0));

            drive.update();

            Pose estimate = drive.getLocalizer().getPoseEstimate();
            Pose predicted = drive.getLocalizer().getPredictedPoseEstimate();

//            telemetry.addData("Pose","("+precision(estimate.getX())+","+precision(estimate.getY())+","+precision(estimate.getHeading())+")");
//            telemetry.addData("Predicted Pose","("+precision(predicted.getX())+","+precision(predicted.getY())+","+precision(predicted.getHeading())+")");

//            telemetry.addData("Pose X", hardware.localizer.getPoseEstimate().getX());
//            telemetry.addData("Pose Y", hardware.localizer.getPoseEstimate().getY());
//            telemetry.addData("Predicted Pose X", hardware.localizer.getPredictedPoseEstimate().getX());
//            telemetry.addData("Predicted Pose Y", hardware.localizer.getPredictedPoseEstimate().getY());
//            telemetry.addData("Pose Heading", hardware.localizer.getPoseEstimate().getHeading());
//            telemetry.addData("velocity", drive.getLocalizer().getVelocity());
//            telemetry.addData("predicted glide X", drive.getLocalizer().glideDelta.getX());
//            telemetry.addData("predicted glide Y", drive.getLocalizer().glideDelta.getY());
//            telemetry.addData("velocity X", drive.getLocalizer().localizer.getPoseVelocity().getX());
//            telemetry.addData("velocity Y", drive.getLocalizer().localizer.getPoseVelocity().getY());

            telemetry.addData("Voltage", voltageSupplier.get());
            telemetry.addData("KS", voltageSupplier.get()/12);
            telemetry.update();
        }
    }
}