package org.firstinspires.ftc.teamcode.opmode.test.ServoTuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;

@Config
@TeleOp(group = "Tune")
public class IntakeDBServoTuner extends LinearOpMode {

    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    DriveBase driveBase = new DriveBase();

    public static double
            DRONE_POS = 0.43,
            INTAKE_ARM_POS = 0.91,
            INTAKE_CHUTE_ARM_POS = 0.905,
            INTAKE_CLIP_POS = 0.45,
            INTAKE_PIXEL_HOLDER_POS = 0.255;

    @Override
    public void runOpMode() {

        driveBase.initDrivebase(hardwareMap);
        intakeSubsystem.initIntake(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            driveBase.DroneServo.setPosition(DRONE_POS);
            intakeSubsystem.IntakeArmServo.setPosition(INTAKE_ARM_POS);
            intakeSubsystem.IntakeChuteArmServo.setPosition(INTAKE_CHUTE_ARM_POS);
            intakeSubsystem.IntakeClipServo.setPosition(INTAKE_CLIP_POS);
            intakeSubsystem.IntakePixelHolderServo.setPosition(INTAKE_PIXEL_HOLDER_POS);
        }
    }
}
