package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.hardware.OuttakeInverseKinematics;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
@Disabled
@TeleOp(name="Backdrop Kinematics")
public class BackdropKinTest extends LinearOpMode {
    double verticalHeight,
            prevVerticalHeight,
            pitchTarget,
            liftTarget,
            robotAngle,
            railTarget;
    ElapsedTime GlobalTimer = new ElapsedTime();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    DriveBase driveBase = new DriveBase(telemetry);
    OuttakeInverseKinematics outtakeInverseKinematics = new OuttakeInverseKinematics();

    @Override
    public void runOpMode() throws InterruptedException {

        outtakeSubsystem.initOuttake(hardwareMap);
        driveBase.initDrivebase(hardwareMap);

        robotAngle = 0;
        verticalHeight = 36;

        waitForStart();
        if (opModeIsActive()) {
            outtakeSubsystem.hardwareSetup();
            driveBase.drivebaseSetup();

            GlobalTimer.reset();
            while (!isStopRequested() && opModeIsActive()) {

                if (gamepad1.left_trigger > 0.2) {
                    driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                }
                else {
                    //in
                    verticalHeight += gamepad1.left_stick_y * 0.0005;

                    pitchTarget = outtakeInverseKinematics.pitchEnd(verticalHeight, robotAngle);
                    liftTarget = outtakeInverseKinematics.slideEnd(verticalHeight, robotAngle);
                   // railTarget = (int)outtakeInverseKinematics.railEnd(prevVerticalHeight,verticalHeight, railTarget, robotAngle);
                }
                outtakeSubsystem.liftTo((int) liftTarget, outtakeSubsystem.liftPosition,1);
                outtakeSubsystem.pitchToInternalPID((int) pitchTarget,1);
                outtakeSubsystem.OuttakeRailServo.setPosition(railTarget);

                telemetry.addData("Vertical Height", verticalHeight);
                telemetry.addData("Pitch Target", pitchTarget);
                telemetry.addData("Lift Target", liftTarget);
                telemetry.addData("Rail Target", railTarget);
                telemetry.update();

            }
        }
    }
}
