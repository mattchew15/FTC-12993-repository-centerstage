package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Localizer;
import org.firstinspires.ftc.teamcode.system.paths.P2P.MecanumDrive;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;

public class AutoP2P extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        DriveBase driveBase = new DriveBase();
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        TimedSupplier<Double> voltageSupplier = new TimedSupplier<>( voltageSensor::getVoltage, 100);

        MecanumDrive drive = new MecanumDrive(
                driveBase.FL, driveBase.FR, driveBase.BL, driveBase.BR,
                MecanumDrive.RunMode.P2P,voltageSupplier);
        drive.setLocalizer(new Localizer(hardwareMap, new Pose(0, 0, Math.toRadians(90))));

        waitForStart();
        while(!isStopRequested() && opModeIsActive())
        {
            drive.setTargetPose(new Pose(10, 0, Math.toRadians(90)));
            drive.update();
        }
    }
}
