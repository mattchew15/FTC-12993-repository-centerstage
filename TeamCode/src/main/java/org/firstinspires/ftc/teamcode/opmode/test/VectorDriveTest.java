package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Localizer;
import org.firstinspires.ftc.teamcode.system.paths.P2P.MecanumDrive;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Vector;

@TeleOp(name = "VectorDriveTest", group = "Test")
public class VectorDriveTest extends LinearOpMode
{

    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException
    {
        DriveBase driveBase = new DriveBase(telemetry);
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        TimedSupplier<Double> voltageSupplier = new TimedSupplier<>( voltageSensor::getVoltage, 100);

        drive = new MecanumDrive(
                driveBase.FL, driveBase.FR, driveBase.BL, driveBase.BR,
                MecanumDrive.RunMode.P2P, voltageSupplier);
        drive.setLocalizer(new Localizer(hardwareMap, new Pose(0, 0, Math.toRadians(90)), this));
        waitForStart();
        while (opModeIsActive())
        {
            //drive.setTargetVector(new Vector(0, 1, 0));
            drive.setTargetPose(new Pose(0, 24, Math.toRadians(90)));
            drive.update();
            telemetry.addData("Position", drive.getLocalizer().getPoseEstimate());
            telemetry.update();
        }
    }

}
