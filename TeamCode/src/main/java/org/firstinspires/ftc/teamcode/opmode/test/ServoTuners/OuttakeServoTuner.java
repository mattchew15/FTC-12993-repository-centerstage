package org.firstinspires.ftc.teamcode.opmode.test.ServoTuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@Config
@TeleOp(group = "Tune")
public class OuttakeServoTuner extends LinearOpMode {

    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();

    public static double
            OUTTAKE_PITCH_POS = 0.5,
            MINI_TURRET_POS = 0.5,
            OUTTAKE_RAIL_POS = 0.5,
            OUTTAKE_ARM_POS = 0.5,
            PIVOT_POS = 0.5,
            GRIPPER_TOP_POS = 0.5,
            GRIPPER_BOTTOM_POS = 0.5;

    @Override
    public void runOpMode() {

        outtakeSubsystem.initOuttake(hardwareMap);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {

            /*
            outtakeSubsystem.OuttakeArmServo.setPosition(OUTTAKE_ARM_POS);
            telemetry.addData("PmW range", outtakeSubsystem.OuttakeArmServo.getPwmRange().usFrame);
            telemetry.addData("Pmw low", outtakeSubsystem.OuttakeArmServo.getPwmRange().usPulseLower);
            telemetry.addData("Pmw low", outtakeSubsystem.OuttakeArmServo.getPwmRange().usPulseUpper);
            telemetry.update();

             */

            outtakeSubsystem.OuttakePitchServo.setPosition(OUTTAKE_PITCH_POS);
            outtakeSubsystem.MiniTurretServo.setPosition(MINI_TURRET_POS);
            outtakeSubsystem.OuttakeRailServo.setPosition(OUTTAKE_RAIL_POS);
            outtakeSubsystem.OuttakeArmServo.setPosition(OUTTAKE_ARM_POS);
            outtakeSubsystem.PivotServo.setPosition(PIVOT_POS);
            outtakeSubsystem.GripperTopServo.setPosition(GRIPPER_TOP_POS);
            outtakeSubsystem.GripperBottomServo.setPosition(GRIPPER_BOTTOM_POS);


        }
    }
}
