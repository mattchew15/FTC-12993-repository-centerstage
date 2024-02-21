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
            DroneServo,
            IntakeArmServo,
            IntakeChuteArmServo,
            IntakeClipServo,
            IntakePixelHolderServo,
            OuttakeArmServoLeft,
            OuttakeArmServoRight,
            MiniTurretServo,
            PivotServo,
            GripperTopServo,
            GripperBottomServo,
            OuttakeRailServo;

    @Override
    public void runOpMode() {

        outtakeSubsystem.initOuttake(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);
        driveBase.initDrivebase(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            intakeSubsystem.IntakeArmServo.setPosition(INTAKE_ARM_POS);
            intakeSubsystem.IntakeChuteArmServo.setPosition(INTAKE_CHUTE_ARM_POS);
            intakeSubsystem.IntakeClipServo.setPosition(INTAKE_CLIP_POS);
            intakeSubsystem.IntakePixelHolderServo.setPosition(INTAKE_PIXEL_HOLDER_POS);
           // outtakeSubsystem.OuttakeArmServoLeft.setPosition(OUTTAKE_ARM_POS);
           // outtakeSubsystem.OuttakeArmServoRight.setPosition(OUTTAKE_ARM_POS);
           // outtakeSubsystem.MiniTurretServo.setPosition(MINI_TURRET_POS);
           // outtakeSubsystem.PivotServo.setPosition(PIVOT_POS);
           // outtakeSubsystem.GripperTopServo.setPosition(GRIPPER_TOP_POS);
           // outtakeSubsystem.GripperBottomServo.setPosition(GRIPPER_BOTTOM_POS);
            driveBase.ClimbHolderServo.setPosition(CLIMB_HOLDER_POS);
            driveBase.DroneServo.setPosition(DRONE_POS);
        }
    }
}
