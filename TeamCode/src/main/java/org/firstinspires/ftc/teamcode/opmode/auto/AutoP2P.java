package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Localizer;
import org.firstinspires.ftc.teamcode.system.paths.P2P.MecanumDrive;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;
import org.firstinspires.ftc.teamcode.system.paths.P2P.TwoTrackingWheelLocalizer;

@Autonomous(name = "P2p test")
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
                MecanumDrive.RunMode.P2P, voltageSupplier);
        drive.setLocalizer(new Localizer(hardwareMap, new Pose(0, 0, Math.toRadians(90)), this));

        //TwoTrackingWheelLocalizer twoTrackingWheelLocalizer = new TwoTrackingWheelLocalizer(hardwareMap)

        waitForStart();
        while(!isStopRequested() && opModeIsActive())
        {
            telemetry.addData("TargetPose", drive.getTargetPose());
            telemetry.addData("PoseEstimate", drive.getLocalizer().getPoseEstimate());
            telemetry.addData("Predicted PoseEstimate", drive.getLocalizer().getPredictedPoseEstimate());
            telemetry.addData("Is reached", drive.reachedTarget(0.5));
            telemetry.addData("DriveTrain Velocity", drive.getLocalizer().driveTrainVelocity);

            telemetry.addLine();
            telemetry.addData("RunMode", drive.getRunMode());
            telemetry.addData("PowerVector", drive.powerVector);
            telemetry.addData("PowerVector X", drive.powerVector.getX());
            telemetry.addData("PowerVector Y", drive.powerVector.getY());
            telemetry.addData("PowerVector Z", drive.powerVector.getZ());
            telemetry.addData("Gliding vector", drive.getLocalizer().getGlideDelta());

            telemetry.addLine();
            telemetry.addData("Voltage", drive.getVoltage());
            telemetry.addData("FL", drive.FLPower);
            telemetry.addData("FR", drive.FRPower);
            telemetry.addData("BL", drive.BLPower);
            telemetry.addData("BR", drive.BRPower);
            telemetry.update();

            drive.setTargetPose(new Pose(0, 10, Math.toRadians(180)));
            //if (drive.reachedTarget(0.5)) drive.setTargetPose(drive.getLocalizer().getPoseEstimate());
            drive.update();
        }
    }
}
