package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;

@Config
@TeleOp(group = "Tune")
public class ServoTuner extends LinearOpMode {

    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
    DriveBase driveBase = new DriveBase();

    public static double
            DRONE_POS = 0,
            INTAKE_ARM_POS = 0,
            INTAKE_CHUTE_ARM_POS = 0,
            INTAKE_CLIP_POS = 0,
            INTAKE_PIXEL_HOLDER_POS = 0,
            OUTTAKE_PITCH_POS = 0,
            MINI_TURRET_POS = 0,
            OUTTAKE_RAIL_POS = 0,
            OUTTAKE_ARM_POS = 0,
            PIVOT_POS = 0,
            GRIPPER_TOP_POS = 0,
            GRIPPER_BOTTOM_POS = 0;

    @Override
    public void runOpMode() {

        driveBase.initDrivebase(hardwareMap);
        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            driveBase.DroneServo.setPosition(DRONE_POS);
            intakeSubsystem.IntakeArmServo.setPosition(INTAKE_ARM_POS);
            intakeSubsystem.IntakeChuteArmServo.setPosition(INTAKE_CHUTE_ARM_POS);
            intakeSubsystem.IntakeClipServo.setPosition(INTAKE_CLIP_POS);
            intakeSubsystem.IntakePixelHolderServo.setPosition(INTAKE_PIXEL_HOLDER_POS);
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
